/*
 * jirs40.c
 *
 *  Created on: Oct 8, 2024
 *      Author: JDT
 * 
 * Calling sequence is 
 * 1. JIRS40_Setup - Initializes 1-wire interface
 * 2. JIRS40_Ambient() - starts conversion and result is ready after ~150-200ms
 * 3. JIRS40_Object() - samples the JIRS40 Analog Object voltage output
 * 4. JIRS40_Ambient() - returns the result from first call.
 * 5. JIRS40_Convert() - convert raw object voltage to a temperature in celsius.
 * 
 * With a RTOS a setup could be:
 * 1 thread calling JIRS40_Ambient() and updating the ambient temperature (at desired rate, but 5-7 Hz is max)
 * 1 thread calling JIRS40_Object() and updating raw object values. Then JIRS40_Convert() convert whenever raw object value 
 * or ambient temperature has changed.
 */

#include "jirs40.h"
#include "helper_functions.h"
#include "lptim.h"				 //to gain access to timer (htim21 which is defined below)
#include "adc.h"				//access to ADC data and handles
#include "tim.h"				//access to timer and handles
#include <stdlib.h>

//The ADC channel where analog output of JIRS40 is connected
#define JIRS40_OBJECT_ADC_CHANNEL ADC_CHANNEL_4
//Timer handle - Should run at 1MHz to give proper timings
TIM_HandleTypeDef * TIM = &htim21;
//Define the 1-wire GPIO port and pin
GPIO_TypeDef *D0_PORT = JIRS40_DIO_GPIO_Port;
uint16_t D0_PIN = JIRS40_DIO_Pin;
//Define the bit position for D0_PIN in GPIO MODE register
#define GPIO_MODE_SHIFT 10

//Function prototypes
static void SelectChannel(MeasureType type);
static void jirs40_Init();
static void jirs40_WriteByte(uint8_t ascii);
static uint8_t jirs40_ReadByte();
static JIRS40_Meas_State jirs40_Convert_And_Read_Ambient(float *value);

JIRS40_Meas_State measureState = JIRS40_Conversion_Start;

void JIRS40_Setup()
{
	jirs40_Init();
}

static void SelectChannel(MeasureType type)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = JIRS40_OBJECT_ADC_CHANNEL;
	//Clear all channels
	(&hadc)->Instance->CHSELR &= ~(uint32_t)(ADC_CHANNEL_MASK);
	/* Enable selected channels */
	(&hadc)->Instance->CHSELR |= (uint32_t)(sConfig.Channel & ADC_CHANNEL_MASK);
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		//Handle error
	}
}

void JIRS40_Object(uint16_t *signal)
{
	SelectChannel(Object);
	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1);
	uint32_t value = HAL_ADC_GetValue(&hadc);
	*signal = (value);
}

/*
Inputs:

Output:
signal, in centigrades celsius
*/
void JIRS40_Ambient(int *signal, bool *result)
{
	//Measure ambient using DIO.
	JIRS40_Meas_State readResult = jirs40_Convert_And_Read_Ambient(&jirs40_ambient);
	if(readResult == JIRS40_Read_Start)
	{
		*result = false;
		return;
	}
	else if(readResult == JIRS40_Meas_Error)
	{
		//Handle error
		*result = false;
		return;
	}

	uint16_t temp = (uint16_t)(100*jirs40_ambient);
	*signal = temp;
	*result = true;
}

/*
 * Function to convert raw 16-bit adc value to a temperature.
 * Argument
 * raw_adc value (16-bits)
 * ambient temperature (in centigrades, 10 degrees celsius means 1000 centrigrades)
 * output: the calculated object temperature
 */
void JIRS40_Convert(uint16_t raw_adc, uint16_t ambient, int *output)
{
	//Adc value is in 16 bit
	int x = ambient;
	int y = raw_adc;

	//Find the closest ambient temperature (to know which tables to search in)
	int ambient_integer = ambient / 100;
	int x1 = ambient_integer - AMBIENT_START;
	int x2 = ambient_integer + 1 - AMBIENT_START;

	if (x1 < 0 || x1 > NO_AMBIENT || x2 < 0 || x2 > NO_AMBIENT)
	{
		*output = 0;
		return;
	}

	//Search the arrays for the first match
	int Q11 = 0, y1 = 0;
	int Q12 = 0, y2 = 0;
	int Q21 = 0, y3 = 0;
	int Q22 = 0, y4 = 0;

	bool result1 = FindAdjacent(&jirs40_lookup[x1][0], y, &Q11, &Q12, &y1, &y2,NO_OBJECT);
	bool result2 = FindAdjacent(&jirs40_lookup[x2][0], y, &Q21, &Q22, &y3, &y4,NO_OBJECT);

	if(!result1 || !result2)
	{
		//Something went wrong.
		*output = 0;
		return;
	}
	//Offset object list values
	Q11 = (Q11 + OBJECT_START) * 100;
	Q12 = (Q12 + OBJECT_START) * 100;
	Q21 = (Q21 + OBJECT_START) * 100;
	Q22 = (Q22 + OBJECT_START) * 100;
	//Offset ambient values
	x1 = (x1 + AMBIENT_START) * 100;
	x2 = (x2 + AMBIENT_START) * 100;

	//Interpolate first list
	int first = linear(y, y1, y2, Q11, Q12);
	//Interpolate second list
	int second = linear(y, y3, y4, Q21, Q22);
	//Interpolate in x-direction
	int result = linear(x, x1, x2, first, second);

	*output = result;
}

#define OPTIMIZE_LEVEL 3
typedef enum
{
	Input = 0,
	Output = 1,
}IO;

typedef enum
{
	Low = 0,
	High = 1,
}Level;

static inline void __delay_init()
{
	MicroSecondTimer_Start();
}


__attribute__ ((optimize(OPTIMIZE_LEVEL))) static inline void __delay_us(volatile uint32_t microseconds)
{
	//Timer should run at 1MHz	
	taskENTER_CRITICAL();
	TIM->Instance->CNT = 0;
	while ((TIM->Instance->CNT) < microseconds);
	taskEXIT_CRITICAL();
}

__attribute__ ((optimize(OPTIMIZE_LEVEL))) static inline void delay_us(uint32_t usdelay)
{
	__delay_us(usdelay);
}

__attribute__ ((optimize(OPTIMIZE_LEVEL))) static Level GetLevel()
{
	if(HAL_GPIO_ReadPin(D0_PORT,D0_PIN) == GPIO_PIN_SET)
	{
		return High;
	}
	else
	{
		return Low;
	}
}

__attribute__ ((optimize(OPTIMIZE_LEVEL))) static void SetLevel(Level level)
{
	switch(level)
	{
	case Low:
		D0_PORT->ODR &= ~D0_PIN;
		break;
	case High:
		D0_PORT->ODR |= D0_PIN;
		break;
	}
}

__attribute__ ((optimize(OPTIMIZE_LEVEL))) static void SetDir(IO io)
{
	switch(io)
	{
	case Input:
		//Clear bits
		D0_PORT->MODER &= ~(0x03 << GPIO_MODE_SHIFT);
		break;
	case Output:
		//Set bits to 01
		D0_PORT->MODER |= (0x01 << GPIO_MODE_SHIFT);
		break;
	}
}

//Initializes 1-wire interface
void jirs40_Init()
{
	__delay_init();
	SetLevel(High);
	SetDir(Output);
	SetDir(Input);
}

/* Resets the device and makes it ready for communincation.
 * Returns 0 if failure
 * Returns 1 if success
 */
uint8_t jirs40_Reset()
{
	uint32_t i = 0;
	SetDir(Output);
	SetLevel(Low);
	delay_us(500);
	SetDir(Input);
	delay_us(1);
	while(GetLevel() == High)
	{
		i++;
		if(i>1000)
		{
			return 0;
		}
	}
	while(GetLevel() == Low);
	return 1;
}

void jirs40_WriteByte(uint8_t ascii)
{
	uint j;
	for(j=0; j<8; j++)
	{
		SetDir(Output);
		SetLevel(Low);
		delay_us(7);
		if(ascii & 0x01)
			SetLevel(High);
		else
			SetLevel(Low);
		delay_us(50);
		SetDir(Input);
		ascii >>= 1;
		//Small delays may be required to trim against spec, this is @32MHz CPU
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
		asm("NOP");
	}

}

uint8_t jirs40_ReadByte()
{
	uint8_t byte = 0;
	uint8_t bi = 0;
	uint j;
	for(j=8; j>0; j--)
	{
		SetLevel(Low);
		SetDir(Output);
		//Small delays may be required to trim against spec, this is @32MHz CPU
		asm("NOP");
		asm("NOP");
		asm("NOP");
		SetDir(Input);
		bi = GetLevel();     //Read Data from bus, LSB in first
		/*move byte 1-bit to right，& move bi 7-bit to left*/
		byte = (byte >> 1) | (bi << 7);
		delay_us(48);
	}
	return byte;
}

uint8_t crc8_update(uint8_t crc, uint8_t data) {
	uint8_t i;

	crc = crc ^ data;

	for (i = 0; i < 8; ++i) {
		if (crc & 0x01) {
			crc = (crc >> 1) ^ 0x8c;
		} else {
			crc >>= 1;
		}
	}

	return crc;
}

JIRS40_Error CT1830ReadTemp(uint16_t *result)
{
	#define BYTE_SIZE 8
	uint8_t bytes[BYTE_SIZE];

	uint8_t crc_calc = 0x00;

	if(jirs40_Reset())
	{
		delay_us(200);
		jirs40_WriteByte(0xcc); //Skip ROM ID
		delay_us(200);
		jirs40_WriteByte(0xbe); //Read register
		delay_us(200);

		//Temperature data is register 0 and 1. But read all 8 and the ninth is crc
		for(int i = 0; i <BYTE_SIZE; ++i)
		{
			bytes[i] = jirs40_ReadByte();
			delay_us(200);
			crc_calc = crc8_update(crc_calc, bytes[i]);
		}
		*result = (bytes[1] << 8) + bytes[0];
		delay_us(200);
		//read crc register
		uint8_t crc  = jirs40_ReadByte();
		if(crc != crc_calc)
		{
			//Handle error
			return JIRS40_ERROR_READ;
		}
		return JIRS40_ERROR_NONE;
	}
	return JIRS40_ERROR_INIT;
}

JIRS40_Error CT1830_Start_Conver(void)
{
	if(jirs40_Reset())
	{
		delay_us(50);
		jirs40_WriteByte(0xcc);
		delay_us(50);
		jirs40_WriteByte(0x44);
		return JIRS40_ERROR_NONE;
	}
	return JIRS40_ERROR_INIT;
}

/*
* Convert binary data to an ambient temperature in celsius
*/
float Convert2temp(unsigned int temp)
{
	uint8_t neg_symbol = 0;
	float tmp;
	if (temp > 0x7fff)
	{
		neg_symbol = 1;
		temp = ~temp;
		temp &= 0x7fff;
		temp ++;
	}

	if (neg_symbol)
		tmp = (double)temp * -7.8125 / 1000.0;
	else
		tmp = (double)temp * 7.8125 / 1000.0;
	return tmp;
}

//Define restart to trigger a new direcly after previous read
//#define JIRS40_RESTART_CONVERSION 1

JIRS40_Meas_State jirs40_Convert_And_Read_Ambient(float *value)
{
	uint16_t tempdata=0;
	JIRS40_Error error = JIRS40_ERROR_NONE;
	*value = -100.0;

	if(measureState == JIRS40_Read_Done)
	{
		measureState = JIRS40_Conversion_Start;
	}

	switch(measureState)
	{
		case JIRS40_Conversion_Start:
		{
			taskENTER_CRITICAL(); //Disable interrupts that may interfere with timing
			error = CT1830_Start_Conver();
			taskEXIT_CRITICAL();
			measureState = JIRS40_Read_Start;
			break;
		}
		case JIRS40_Read_Start:
		{
			taskENTER_CRITICAL(); //Disable interrupts that may interfere with timing
			error = CT1830ReadTemp(&tempdata);
			taskEXIT_CRITICAL();
			if(error == JIRS40_ERROR_NONE)
			{
				*value = Convert2temp(tempdata);
				measureState = JIRS40_Read_Done;
#ifdef JIRS40_RESTART_CONVERSION
				taskENTER_CRITICAL();
				error = CT1830_Start_Conver();
				taskEXIT_CRITICAL();
				measureState = JIRS40_Read_Start;
				return JIRS40_Read_Done;
#endif
				break;
			}
			else
			{
				//Something went wrong. Let is fall through and handle error
				measureState = JIRS40_Meas_Error;
			}
			break;
		}
		default:
		{
			//Do some reset!
			measureState = JIRS40_Conversion_Start;
			break;
		}

	}
	if(error != JIRS40_ERROR_NONE)
		log_trace("JIRS40 error:%d",error);
	return measureState;
}

