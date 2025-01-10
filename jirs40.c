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
 * 
 * This example is made for a STM32 Platform. Requires modification if other platform is used!
 */

#include "jirs40.h"
#include "adc.h"				//access to ADC data and handles
#include "tim.h"				//to gain access to timer (htim21 which is defined below)
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
static void jirs40_init_parameters(JIRS40_Calibration * c, const JIRS40_Cal_Param *input);
JIRS40_Meas_State measureState = JIRS40_Conversion_Start;
const float ADC_REFERENCE = 3.3f; //The supply/reference to the ADC module. Could either be dynamically measured or hardcoded
/*
* Calibration points below can be used to updated jirs40_cal struct.
* Update the points and call jirs40_init_parameters(&jirs40_cal, &jirs40_cbt_param_default);
*/
const JIRS40_Cal_Param jirs40_cbt_param_default =
{
	.calibrationP1_Object = 40.0,
	.calibrationP1_Ambient = 23.15,
	.calibrationP1_Voltage = 1.894,
	.calibrationP2_Object = 30.0,
	.calibrationP2_Ambient = 22.96,
	.calibrationP2_Voltage = 1.474,
};

static JIRS40_Calibration jirs40_cal=
{
		.adc_max = 65535,
		.sConP1 = 0,
		.sConP2 = 0,
		//Hardcoded (but can be updated)
		.SystemBias_P1P2 = -1.19925157,
		.SysCal_P1P2 = 4.00492E-10,
		.params  = {0},
};

static JIRS40_Calibration * jirs40_cal = &jirs40_cal;

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
vvoid JIRS40_Convert(uint16_t raw_adc, uint16_t ambient, int *output)
{
	//Adc value is in 16 bit
	float raw = (float)raw_adc;
	float VDD_Measured = ADC_REFERENCE;
	double measured_voltage = VDD_Measured * (raw/jirs40_cal->adc_max);
	double amb = (double)ambient / 100;
	//No longer using lookup table, now calculations instead.

	double temp1 = pow(amb+273.15, 4);
	double temperature = pow(((measured_voltage + jirs40_cal->SystemBias_P1P2)/jirs40_cal->SysCal_P1P2 + temp1), 0.25) - 273.15;

	*output = (int)(temperature*100);
}

static void jirs40_init_parameters(JIRS40_Calibration * c, const JIRS40_Cal_Param *input)
{
	//Update our calculation parameters
	c->adc_max = 65535;
	c->params.calibrationP1_Object = input->calibrationP1_Object;
	c->params.calibrationP1_Ambient = input->calibrationP1_Ambient;
	c->params.calibrationP1_Voltage = input->calibrationP1_Voltage;

	c->params.calibrationP2_Object = input->calibrationP2_Object;
	c->params.calibrationP2_Ambient = input->calibrationP2_Ambient;
	c->params.calibrationP2_Voltage = input->calibrationP2_Voltage;

	//Lets get a pointer to params to avoid bloating calculations below
	JIRS40_Cal_Param *p = &c->params;

	//Calculate new bias etc based upon new data.
	c->sConP1 = pow(p->calibrationP1_Object + 273.15f, 4) - pow(p->calibrationP1_Ambient + 273.15, 4);
	c->sConP2 = pow(p->calibrationP2_Object + 273.15f, 4) - pow(p->calibrationP2_Ambient + 273.15, 4);
	c->SystemBias_P1P2 =(p->calibrationP1_Voltage-(c->sConP1/c->sConP2)*p->calibrationP2_Voltage)/((c->sConP1/c->sConP2)-1);
	c->SysCal_P1P2 = (p->calibrationP2_Voltage + c->SystemBias_P1P2) / c->sConP2;
	//Update global point
	jirs40_cal = c;
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
		//Clear bits - Changes GPIO to input
		D0_PORT->MODER &= ~(0x03 << GPIO_MODE_SHIFT);
		break;
	case Output:
		//Set bits - Changes GPIO to output
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

