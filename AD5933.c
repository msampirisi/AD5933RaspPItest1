/***************************************************************************//**
 *   @file   AD5933.c
 *   @brief  Implementation of AD5933 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "AD5933.h"
#include <math.h>

/******************************************************************************/
/************************** Constants Definitions *****************************/
/******************************************************************************/
const long POW_2_27 = 134217728ul;      // 2 to the power of 27

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned long currentSysClk      = AD5933_INTERNAL_SYS_CLK;
unsigned char currentClockSource = AD5933_CONTROL_INT_SYSCLK;
unsigned char currentGain        = AD5933_GAIN_X1;
unsigned char currentRange       = AD5933_RANGE_2000mVpp;

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Initializes the communication peripheral.
 *
 * @return status - The result of the initialization procedure.
 *                  Example: -1 - I2C peripheral was not initialized.
 *                            0 - I2C peripheral was initialized.
*******************************************************************************/
char AD5933_Init(void)
{
    unsigned char status = -1;
    
    //status = I2C_Init(100000);
    return 1;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue   - Data value to write.
 * @param bytesNumber     - Number of bytes.
 *
 * @return None.
*******************************************************************************/
void AD5933_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber)
{
    unsigned char byte          = 0;
    unsigned char writeData[2]  = {0, 0};
	int status;

    for(byte = 0;byte < bytesNumber; byte++)
    {
        writeData[0] = registerAddress + bytesNumber - byte - 1;
        writeData[1] = (unsigned char)((registerValue >> (byte * 8)) & 0xFF);
        wiringPiI2CWriteReg8(i2cdevice,writeData[0],writeData[1]);
        //printf("\t\tWriting: 0x%x into 0x%x\n",writeData[1],writeData[0]);
        //printf("Status = %d\n",status);
        //printf("writeData[0]=0x%x\n",writeData[0]);
        //printf("writeData[1]=0x%x\n",writeData[1]);
        //I2C_Write(AD5933_ADDRESS, writeData, 2, 1);
    }
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber     - Number of bytes.
 *
 * @return registerValue  - Value of the register.
*******************************************************************************/
unsigned long AD5933_GetRegisterValue(unsigned char registerAddress,
                                      unsigned char bytesNumber)
{
    unsigned long registerValue = 0;
    unsigned char byte          = 0;
    unsigned char writeData[2]  = {0, 0};
    unsigned char readData[2]   = {0, 0};
    int tmp = 0;
    
    for(byte = 0;byte < bytesNumber;byte ++)
    {
        // Read byte from specified registerAddress memory place
		tmp = wiringPiI2CReadReg8(i2cdevice,registerAddress);
		//printf("\t\tReading from Register Address: 0x%02x...0x%02x\n",registerAddress,tmp);
		// Add this temporal value to our registerValue (remembering that
		// we are reading bytes that have location value, which means that
		// each measure we have we not only have to add it to the previous
		// register value but we also but do a bitwise shift (<< 8) by 1 byte
		registerValue = registerValue << 8;
        registerValue += tmp;
        // Update value from registerAddress to read next memory position byte
        registerAddress = registerAddress + 1;
    }
    
    return registerValue;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return None.
*******************************************************************************/
void AD5933_Reset(void)
{
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_LB, 
                            AD5933_CONTROL_RESET | currentClockSource,
                            1);
}

/***************************************************************************//**
 * @brief Selects the source of the system clock.
 *
 * @param clkSource  - Selects the source of the system clock.
 *                     Example: AD5933_CONTROL_INT_SYSCLK
 *                              AD5933_CONTROL_EXT_SYSCLK
 * @param extClkFreq - Frequency value of the external clock, if used.
 *
 * @return None.
*******************************************************************************/
void AD5933_SetSystemClk(char clkSource, unsigned long extClkFreq)
{
    currentClockSource = clkSource;
    if(clkSource == AD5933_CONTROL_EXT_SYSCLK)
    {
        currentSysClk = extClkFreq;                 // External clock frequency
    }
    else
    {
        currentSysClk = AD5933_INTERNAL_SYS_CLK;    // 16 MHz
    }
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_LB, currentClockSource, 1);
}


/***************************************************************************//**
 * @brief Selects the range and gain of the device.
 *  
 * @param range - Range option.
 *                Example: AD5933_RANGE_2000mVpp
 *                         AD5933_RANGE_200mVpp
 *                         AD5933_RANGE_400mVpp

 *                         AD5933_RANGE_1000mVpp
 * @param gain  - Gain option.
 *                Example: AD5933_GAIN_X5
 *                         AD5933_GAIN_X1
 *
 * @return None.
*******************************************************************************/
void AD5933_SetRangeAndGain(char range, char gain)
{
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                         AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_NOP) |
                         AD5933_CONTROL_RANGE(range) | 
                         AD5933_CONTROL_PGA_GAIN(gain),
                         1);
    /* Store the last settings made to range and gain. */
    currentRange = range;
    currentGain = gain;
}

/***************************************************************************//**
 * @brief Reads the temperature from the part and returns the data in
 *        degrees Celsius.
 *
 * @return temperature - Temperature.
*******************************************************************************/
float AD5933_GetTemperature(void)
{
    float         temperature = 0;
    unsigned char status      = 0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                         AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_MEASURE_TEMP) |
                         AD5933_CONTROL_RANGE(currentRange) | 
                         AD5933_CONTROL_PGA_GAIN(currentGain),                             
                         1);
    while((status & AD5933_STAT_TEMP_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    
    temperature = AD5933_GetRegisterValue(AD5933_REG_TEMP_DATA,2);
    if(temperature < 8192)
    {
        temperature /= 32;
    }
    else
    {
        temperature -= 16384;
        temperature /= 32;
    }
    
    return temperature;
}

/***************************************************************************//**
 * @brief Configures the sweep parameters: Start frequency, Frequency increment
 *        and Number of increments.
 *
 * @param startFreq - Start frequency in Hz;
 * @param incFreq   - Frequency increment in Hz;
 * @param incNum    - Number of increments. Maximum value is 511(0x1FF).
 *
 * @return None.
*******************************************************************************/
void AD5933_ConfigSweep(unsigned long  startFreq,
                        unsigned long  incFreq,
                        unsigned short incNum)
{
    unsigned long  startFreqReg = 0;
    unsigned long  incFreqReg   = 0;
    unsigned short incNumReg    = 0;
    
    // Ensure that incNum is a valid data. 
    if(incNum > AD5933_MAX_INC_NUM)
    {
        incNumReg = AD5933_MAX_INC_NUM;
    }
    else
    {
        incNumReg = incNum;
    }
    
    // Convert users start frequency to binary code. //
    startFreqReg = (unsigned long)((double)startFreq * 4 / currentSysClk *
                                   POW_2_27);
   
    // Convert users increment frequency to binary code. //
    incFreqReg = (unsigned long)((double)incFreq * 4 / currentSysClk * 
                                 POW_2_27);
    
    
    printf("Configuring Sweeping Parameters:\n\tStarting Freq = %lu (0x%06x)\n",startFreq,startFreqReg);
    printf("\tIncrement Frequency = %lu (0x%06x)\n",incFreq,incFreqReg);
    printf("\tNumber of Points = %d (0x%04x)\n",incNum,incNum);
    
    // Configure the device with the sweep parameters. //
    AD5933_SetRegisterValue(AD5933_REG_FREQ_START,
                            startFreqReg,
                            3);
    AD5933_SetRegisterValue(AD5933_REG_FREQ_INC,
                            incFreqReg,
                            3);
    AD5933_SetRegisterValue(AD5933_REG_INC_NUM,
                            incNumReg,
                            2);
}

/***************************************************************************//**
 * @brief Starts the sweep operation.
 *
 * @return None.
*******************************************************************************/
void AD5933_StartSweep(void)
{
    unsigned char status = 0;
    
    // put AD5933 in standby mode (required, see datasheet)
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY) |
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
	// Reset device
    AD5933_Reset();
    
    // Initialize sweep with start frequency (this does not start the sweep,
    // just initializes some parameters)
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                       AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_INIT_START_FREQ)|
                       AD5933_CONTROL_RANGE(currentRange) | 
                       AD5933_CONTROL_PGA_GAIN(currentGain),
                       1);
    
    // Start the Sweep
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                       AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_START_SWEEP) | 
                       AD5933_CONTROL_RANGE(currentRange) | 
                       AD5933_CONTROL_PGA_GAIN(currentGain),
                       1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    };
}

/******************************************************************************
* @brief Calculate gain factor
*
* @param calibrationImpedance - Known value of connected impedance for calibration.
*
* @param freqFunction - Select Repeat Frequency Sweep.
*
* @return gainFactor.
******************************************************************************/
double AD5933_CalculateGainFactor(unsigned long calibrationImpedance,
							   char freqFunction)
{
	double       gainFactor = 0;
	double       magnitude  = 0;
	int          status     = 0;
	signed short realData   = 0;
	signed short imgData    = 0;

	// Repeat frequency sweep with last set parameters
	AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
							AD5933_CONTROL_FUNCTION(freqFunction)|
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
							1);

	// Wait for data received to be valid
	while((status & AD5933_STAT_DATA_VALID) == 0)
	{
		status = AD5933_GetRegisterValue(AD5933_REG_STATUS, 1);
	}


	// Get real and imaginary reg parts
	signed short RealPart = 0;
	signed short ImagPart = 0;
	unsigned char byte          = 0;
	int tmp = 0;
	
	unsigned char registerAddress = AD5933_REG_REAL_DATA;
	for(byte = 0;byte < 2;byte ++)
	{
		// Read byte from specified registerAddress memory place
		tmp = wiringPiI2CReadReg8(i2cdevice,registerAddress);
		printf("\t\tReading from Register Address: 0x%02x...0x%02x\n",registerAddress,tmp);
		// Add this temporal value to our registerValue (remembering that
		// we are reading bytes that have location value, which means that
		// each measure we have we not only have to add it to the previous
		// register value but we also but do a bitwise shift (<< 8) by 1 byte
		RealPart = RealPart << 8;
		RealPart += tmp;
		// Update value from registerAddress to read next memory position byte
		registerAddress = registerAddress + 1;
	}
	
	
	registerAddress = AD5933_REG_IMAG_DATA;
	for(byte = 0;byte < 2;byte ++)
	{
		// Read byte from specified registerAddress memory place
		tmp = wiringPiI2CReadReg8(i2cdevice,registerAddress);
		printf("\t\tReading from Register Address: 0x%02x...0x%02x\n",registerAddress,tmp);
		// Add this temporal value to our registerValue (remembering that
		// we are reading bytes that have location value, which means that
		// each measure we have we not only have to add it to the previous
		// register value but we also but do a bitwise shift (<< 8) by 1 byte
		ImagPart = ImagPart << 8;
		ImagPart += tmp;
		// Update value from registerAddress to read next memory position byte
		registerAddress = registerAddress + 1;
	}
	
	
	magnitude = sqrt((RealPart * RealPart) + (ImagPart * ImagPart));
	


	/*
	// Read real and imaginary data
	realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
	imgData  = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);

	// Calculate magnitude
	magnitude = sqrtf((realData * realData) + (imgData * imgData));
	*/
	
	// Calculate gain factor
	gainFactor = 1 / (magnitude * calibrationImpedance);

	printf("Calibration Step:\n\tR=%hi\n\tI=%hi\n\t|Z|=%f\n",realData,imgData,magnitude);

	return(gainFactor);
}

/******************************************************************************
* @brief Calculate impedance.
*
* @param gainFactor - Gain factor calculated using a known impedance.
*
* @param freqFunction - Select Repeat Frequency Sweep.
*
* @return impedance.
******************************************************************************/
double AD5933_CalculateImpedance(double gainFactor,
								 char freqFunction)
{
	signed short realData   = 0;
	signed short imgData    = 0;
	double       magnitude  = 0;
	double       impedance  = 0;
	int          status     = 0;

	// Repeat frequency sweep with last set parameters
	AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
							AD5933_CONTROL_FUNCTION(freqFunction)|
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
							1);



	// Wait for data received to be valid
	while((status & AD5933_STAT_DATA_VALID) == 0)
	{
		status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
	}
	
	
	// Get real and imaginary reg parts
	signed short RealPart = 0;
	signed short ImagPart = 0;
	unsigned char byte          = 0;
	int tmp = 0;
	
	unsigned char registerAddress = AD5933_REG_REAL_DATA;
	for(byte = 0;byte < 2;byte ++)
	{
		// Read byte from specified registerAddress memory place
		tmp = wiringPiI2CReadReg8(i2cdevice,registerAddress);
		//printf("\t\tReading from Register Address: 0x%02x...0x%02x\n",registerAddress,tmp);
		// Add this temporal value to our registerValue (remembering that
		// we are reading bytes that have location value, which means that
		// each measure we have we not only have to add it to the previous
		// register value but we also but do a bitwise shift (<< 8) by 1 byte
		RealPart = RealPart << 8;
		RealPart += tmp;
		// Update value from registerAddress to read next memory position byte
		registerAddress = registerAddress + 1;
	}
	
	
	registerAddress = AD5933_REG_IMAG_DATA;
	for(byte = 0;byte < 2;byte ++)
	{
		// Read byte from specified registerAddress memory place
		tmp = wiringPiI2CReadReg8(i2cdevice,registerAddress);
		//printf("\t\tReading from Register Address: 0x%02x...0x%02x\n",registerAddress,tmp);
		// Add this temporal value to our registerValue (remembering that
		// we are reading bytes that have location value, which means that
		// each measure we have we not only have to add it to the previous
		// register value but we also but do a bitwise shift (<< 8) by 1 byte
		ImagPart = ImagPart << 8;
		ImagPart += tmp;
		// Update value from registerAddress to read next memory position byte
		registerAddress = registerAddress + 1;
	}
	
	
	magnitude = sqrt((RealPart * RealPart) + (ImagPart * ImagPart));
	
	//printf("Z = %hi + %hi*i ... |Z| = %f\n",RealPart,ImagPart,magnitude);
	
	return magnitude;
	

	//// Read real and imaginary data
	//realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
	//imgData  = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);

	//// Calculate magnitude
	//magnitude = sqrtf((realData * realData) + (imgData * imgData));

	//// Calculate impedance
	//impedance = 1 / (magnitude * gainFactor / 1000000000);

	//return(impedance);
}







/***************************************************************************//**
 * @brief Reads the real and the imaginary data and calculates the Gain Factor.
 *
 * @param calibrationImpedance - The calibration impedance value.
 * @param freqFunction         - Frequency function.
 *                               Example: AD5933_FUNCTION_INC_FREQ - Increment 
                                          freq.;
 *                                        AD5933_FUNCTION_REPEAT_FREQ - Repeat 
                                          freq..
 *
 * @return gainFactor          - Calculated gain factor.
*******************************************************************************/
/*double AD5933_CalculateGainFactor2(unsigned long calibrationImpedance,
                                  unsigned char freqFunction)
{
    double        gainFactor = 0;
    unsigned long long       magnitude  = 0;
    signed short  realData   = 0;
    signed short  imagData   = 0;
    unsigned char status     = 0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(freqFunction) |
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),    
                            1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
    imagData = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);
    magnitude = sqrt((realData * realData) + (imagData * imagData));
    gainFactor = 1 / (magnitude * calibrationImpedance);

	printf("Calibration Step:\n\tR=%hi\n\tI=%hi\n\t|Z|=%llu\n",realData,imagData,magnitude);

    return gainFactor;
}*/

/***************************************************************************//**
 * @brief Reads the real and the imaginary data and calculates the Impedance.
 *
 * @param gainFactor   - The gain factor.
 * @param freqFunction - Frequency function.
 *                       Example: AD5933_FUNCTION_INC_FREQ - Increment freq.;
 *                                AD5933_FUNCTION_REPEAT_FREQ - Repeat freq..
 *
 * @return impedance   - Calculated impedance.
*******************************************************************************/
/*double AD5933_CalculateImpedance2(double gainFactor,
                                 unsigned char freqFunction)
{
    signed short  realData  = 0;
    signed short  imagData  = 0;
    double        magnitude = 0;
    double        impedance = 0;
    unsigned char status    = 0;
    
    AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
                            AD5933_CONTROL_FUNCTION(freqFunction) | 
                            AD5933_CONTROL_RANGE(currentRange) | 
                            AD5933_CONTROL_PGA_GAIN(currentGain),
                            1);
    status = 0;
    while((status & AD5933_STAT_DATA_VALID) == 0)
    {
        status = AD5933_GetRegisterValue(AD5933_REG_STATUS,1);
    }
    realData = AD5933_GetRegisterValue(AD5933_REG_REAL_DATA,2);
    imagData = AD5933_GetRegisterValue(AD5933_REG_IMAG_DATA,2);
    magnitude = sqrtf((realData * realData) + (imagData * imagData));
    
    impedance =  1 / (magnitude * gainFactor);
    
    return impedance;    
}*/

/**************************************************************************//**
 * @brief Set AD5933 to standby mode
 * 
 * @return none
 * 
*******************************************************************************/
void AD5933_SetToStandBy(void)
{
	AD5933_SetRegisterValue(AD5933_REG_CONTROL_HB,
								AD5933_CONTROL_FUNCTION(AD5933_FUNCTION_STANDBY) |
								AD5933_CONTROL_RANGE(currentRange) | 
								AD5933_CONTROL_PGA_GAIN(currentGain),
								1);
								
}






