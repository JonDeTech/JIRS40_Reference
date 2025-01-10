/*
 * jirs40.c
 *
 *  Created on: Oct 8, 2024
 *      Author: JDT
 */

#include "main.h"		//To access ADC handles
#include "jirs40_lookup.h"

typedef enum
{
	JIRS40_Conversion_Start,
	JIRS40_Read_Start,
	JIRS40_Read_Done,
	JIRS40_Meas_Error
} JIRS40_Meas_State;

typedef enum
{
	JIRS40_ERROR_NONE = 0,
	JIRS40_ERROR_INIT = 1,
	JIRS40_ERROR_CONV = 2,
	JIRS40_ERROR_READ = 3,
	JIRS40_ERROR_UNKNOWN = 4,

}JIRS40_Error;

typedef struct
{
	double calibrationP1_Object;
	double calibrationP1_Ambient;
	double calibrationP1_Voltage;
	double calibrationP2_Object;
	double calibrationP2_Ambient;
	double calibrationP2_Voltage;

}JIRS40_Cal_Param;

typedef struct
{
	double adc_max;
	double sConP1;
	double sConP2;
	double SystemBias_P1P2;
	double SysCal_P1P2;
	JIRS40_Cal_Param params;
}JIRS40_Calibration;

//Each "sensor" should contain functions for setup and measure
//This function is called when sensor should be switched
void JIRS40_Setup();
void JIRS40_Object(uint16_t *signal);
void JIRS40_Ambient(int *signal, bool *result);
void JIRS40_Convert(uint16_t raw_adc, uint16_t ambient, int *output);

uint8_t jirs40_Reset();
