/*
 * ADS1299_drivers.c
 *
 *  Created on: Jul 12, 2021
 *      Author: westr
 */
#include "ADS1299_drivers.h"
#include "main.h"
// External variables
DOUT_t DOUT;
/*
 * \brief Temporary functions only for when Stoffe is debugging. Should be removed.
 */
void hodl()
{
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET);
	HAL_Delay(500);
}


/*
 *	\brief Transmits data over SPI to ADS1299. Toggles CS pin
 *			prior and after transmit.
 *	@param data Array of data to be transmitted. If only one byte,
 *	please dereference (&data) parameter when calling function.
 *	@param size Number of bytes in the data
 */
void ADS_Transmit(uint8_t* data, uint16_t size)
{
//	ADS_DRDY_Wait();		//TODO: Visst ska´re va så?
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_SPI_Transmit(&ADS_SPI, (uint8_t *) data, size, 100);
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);


	HAL_Delay(1);	// TODO: This shouldn't be here?

	//	hodl();
}


/*
 * 	\brief Is used to wait for DRDY to go low before sending data
 */
void ADS_DRDY_Wait(){
	while(HAL_GPIO_ReadPin(ADS_DRDY_BUS, ADS_DRDY_PIN) == GPIO_PIN_SET);
//	HAL_Delay(1); // TODO: is this needed?
}

// Test function, remove once all is good
void ADS_device_init()
{

	// wait for oscillator to warm up
	HAL_Delay(1000);

	// testar RESET,  borde väl inte behövas?
	ADS_RESET();

	/*************TEST***********/
	// wait for oscillator to warm up
	HAL_Delay(1000);

	// testar RESET,  borde väl inte behövas?
	ADS_RESET();
	/*************END TEST**************/

	// Är problemet  att VCAP1 < 1.1 V när SDATAC skickas?
//	send_uart("is vcap1>1.1 V?\n", huart2);
//
//	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET);

	// wait for oscillator to warm up
	HAL_Delay(1000);

	ADS_SDATAC();


	HAL_Delay(1000);


//	ADS_STOP();


	//	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);
	//	ADS_WriteReg(ADS_CONFIG1_ADDR, 0x96U);
	//	ADS_WriteReg(ADS_CONFIG1_ADDR, 150);

/*****	OK MED CPOL 1 CPHA 1 ****/

//	ADS_WriteReg(ADS_CH1SET_ADDR, 0xE0U);	// OK E0 och E1, ingen bit försvinner. Då CPOL=1, CPHA=1, dvs spi mode 4..
//
//	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);	// dessa ska testas när read fungerar igen efter CPHA=1
/*********************************/

	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);
//	hodl();

	ADS_WriteReg(ADS_CONFIG3_ADDR, 0xE0U);
//	hodl();
	HAL_Delay(1000);

	ADS_WriteReg(ADS_CONFIG2_ADDR, 0xC0U);

//	hodl();
	ADS_WriteReg(ADS_CONFIG1_ADDR, 0x96U); // Fungerar inte?

//	hodl();
	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);

	ADS_WriteReg(ADS_CH1SET_ADDR, 0x0U);
//	ADS_WriteReg(ADS_CH2SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH3SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH4SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH5SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH6SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH7SET_ADDR, 0x01U);
//	ADS_WriteReg(ADS_CH8SET_ADDR, 0x01U);

	ADS_WriteReg(ADS_CH2SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH3SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH4SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH5SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH6SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH7SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH8SET_ADDR, 0x81U);

//	hodl();

	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);


//	HAL_Delay(2000); //T_SETTLE???
//	ADS_START();
//	HAL_Delay(2000); //T_SETTLE???

	HAL_Delay(1000);
	ADS_RDATAC();
	HAL_Delay(1000);



	uint8_t ads_data[27];
	uint8_t ads_trash[27];
	for(int i = 0; i < 27; i++){
		ads_trash[i]=0;
	}
	while(1)
	{
		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);

//		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
//		HAL_Delay(100);
		hodl();


		ADS_DRDY_Wait();

		ADS_DOUT();
//		ADS_Send();


//		HAL_SPI_TransmitReceive(&hspi3, ads_trash, ads_data, 27, 100);

//		HAL_UART_Transmit(&huart2, ads_data, 27, 100);
			// ONSDAG BÖRJA HÄR

	}



	send_uart("delaying..\n", huart2);
	HAL_Delay(900000);

}

/*
 * 	\brief Powers on the ADS1299 in accordance to the correct power-up sequence.
 * 	It will power up using internal reference, assuming internal clock @ 2.048 MHz
 */
void ADS_PowerOn()
{
	// wait for oscillator to warm up
	HAL_Delay(1000);

	// testar RESET,  borde väl inte behövas?
	ADS_RESET();

	/*************TEST***********/ // remove?
	// wait for oscillator to warm up
//	HAL_Delay(1000);

	// testar RESET,  borde väl inte behövas?
//	ADS_RESET();
	/*************END TEST**************/

	// wait for oscillator to warm up
//	HAL_Delay(1000);

	ADS_SDATAC();

//	HAL_Delay(1000);


	//	Set internal reference
//	ADS_CONFIG3(ADS1299_CONFIG3_INT_REF_BUF_ENABLE);

	// TEMPORARY 2021-10-28. should use int_ref_buf_enable(?)
//	ADS_BIAS_SENS(ADS1299_BIAS_SENSN_BIASN1_ENABLED, ADS1299_BIAS_SENSP_BIASP1_ENABLED);
//	ADS_BIAS_SENS(0, ADS1299_BIAS_SENSP_BIASP1_ENABLED);
//	ADS_CHANNEL(ADS_CH1SET_ADDR, (ADS1299_CHN_POWER_ON | ADS1299_CHN_GAIN_1 | 0b111));

	// Temporary for testing
//	ADS_CHANNEL(ADS_CH1SET_ADDR, (ADS1299_CHN_POWER_ON | ADS1299_CHN_GAIN_1 | ADS1299_CHN_INPUT_NORMAL_ELECTRODE));
//	ADS_CHANNEL(ADS_CH2SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH3SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH4SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH5SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH6SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH7SET_ADDR, ADS1299_CHN_POWER_OFF);
//	ADS_CHANNEL(ADS_CH8SET_ADDR, ADS1299_CHN_POWER_OFF);

//	ADS_CONFIG1(0x96);
//	ADS_CONFIG3(0xE0);
//
//
//
//	ADS_CONFIG1(0b10010011); // DR=011 => 2 kHz

	// Enable internal reference, makes 0 difference
//	ADS_CONFIG3(ADS1299_CONFIG3_INT_REF_BUF_ENABLE);

	// Set CONFIG1 with 2 kHz, assuming f_CLK=2.048 MHz => f_MOD=f_CLK/2, output data rate = f_MOD/512 = 2 kHz or 2 kSPS
//	ADS_CONFIG1(ADS1299_CONFIG1_DAISY_DISABLE | ADS1299_CONFIG1_CLOCK_DISABLE | ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_512); // This should be 0b11010011
//	ADS_CONFIG1(ADS1299_CONFIG1_DAISY_ENABLE | ADS1299_CONFIG1_CLOCK_DISABLE | ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_512); // This should be 0b10010011


//	ADS_ReadReg(ADS_CONFIG1_ADDR, 1);

		// Input shorted. THIS IS ONLY FOR NOISE MEASUREMENT, RITE?
//	ADS_CHANNEL(ADS_CH1SET_ADDR, ADS1299_CHN_GAIN_2 | ADS1299_CHN_INPUT_INPUT_SHORTED); // Should be 0b00010000

		// Input_normal_electrode. Best result so far. I would say this is about 1/10 in success
//	ADS_CHANNEL(ADS_CH1SET_ADDR, ADS1299_CHN_GAIN_2 | ADS1299_CHN_INPUT_NORMAL_ELECTRODE); // Should be 0b00010000


		//	 Bias measurement, this needs to set BIAS_MEASA in CONFIG3 as well! Result is about as useless as they come
//	ADS_CONFIG3(ADS1299_CONFIG3_BIAS_MEAS_ENABLE);
//	ADS_CHANNEL(ADS_CH1SET_ADDR, ADS1299_CHN_GAIN_2 | ADS1299_CHN_INPUT_BIAS_MEASUREMENT); // Should be 0b00010000

//	ADS_ReadReg(ADS_CH1SET_ADDR, 1);	// Check




	// Nebojsa
	ADS_CONFIG1(0b11010000 | ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_1024); // OBS! Config1 needs to be done first as this reset
	ADS_CHANNEL(ADS_CH1SET_ADDR, 0b01100000);

	ADS_CONFIG2(0b11010100);

	ADS_CONFIG3(0b11101100);
	ADS_BIAS_SENS(0b00000001, 0b00000001);

	ADS_ReadReg(ADS_BIAS_SENSN_ADDR, 1);

//	ADS_CHANNEL(ADS_CH1SET_ADDR, 0x05);
//	ADS_CHANNEL(ADS_CH2SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH3SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH4SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH5SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH6SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH7SET_ADDR, (0x05));
//	ADS_CHANNEL(ADS_CH8SET_ADDR, (0x05));


//	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);

	ADS_START();
	HAL_Delay(500);
	ADS_RDATAC();

//	HAL_Delay(99999);
}

/*
 * 	\brief Powers on the ADS1299 in accordance to the correct power-up sequence for test signals on IN1N and IN1P.
 * 	This is used to ensure that the unit is working.
 * 	It will power up using internal reference, assuming internal clock @ 2.048 MHz
 */
void ADSPowerOnTest()
{
	// wait for oscillator to warm up
	HAL_Delay(1000);
	ADS_RESET();
	ADS_SDATAC();
//	ADS_CONFIG1(0b11010000 | ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_1024);
	ADS_CONFIG1(0b11010000 | ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_4096);
	//ADS_CONFIG2(0xC0); // Set INT_CAL to 1 : Test signals are generated internally
	ADS_CONFIG2(0b11010100); // "To enable the internal test signal, you need to set CONFIG2.Bit1 to '1' and CONFIG2.Bit0 to '1'"
	ADS_CHANNEL(ADS_CH1SET_ADDR, 0x05); // Set MUX1 to 101: Test signal, this generates test signal for IN1P and IN1N
	ADS_ReadReg(ADS_CONFIG1_ADDR, 2); // C1= 0b11010100,	C2=0b11000000
	ADS_START();
	HAL_Delay(500);
	ADS_RDATAC();
}


/*
 * 	\brief Read values of ADS1299 register(s).
 * 	@param baseAddr Address of first register to be read.
 * 	@param numOfReg Number of registers to be read following baseAddr, to only read one register, set numOfReg=1
 *
 */
void ADS_ReadReg(uint8_t baseAddr, uint8_t numOfReg)
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
//	HAL_Delay(1);		// Min 6 ns

	uint8_t opcode = ADS_CMD_RREG + baseAddr;

	//	Stop Read Data Continuously
	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);

	ADS_Transmit(&opcode, 1);

	ADS_Transmit(&(uint8_t){numOfReg-1}, 1);

	uint8_t readReg = 0;


	//	TODO: Ta bort när denna är helt ok! Nä.... varför skulle jag göra det? Vi vill ju se resultatet, dumbom
	for(int i = 0; i < numOfReg; i++)
	{
		//ADS_DRDY_Wait();
//		ADS_Transmit(&(uint8_t){0x00}, 1);
		HAL_SPI_TransmitReceive(&ADS_SPI, &(uint8_t){0x00}, &readReg, 1, 100);
		HAL_Delay(1); // Vadan denna delay??
	}

//		_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
//	HAL_Delay(1);		// Min 6 ns

}

// Remove? A bit redundant
void ADS_RReg(uint8_t baseAddr)
{
	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns

	uint8_t opcode = (ADS_CMD_RREG | baseAddr);
	ADS_Transmit(&opcode, 1);

	// Read one register
	ADS_Transmit(&(uint8_t){0x00}, 1);

	//	Receive
	ADS_DRDY_Wait();
	ADS_Transmit(&(uint8_t){0x00}, 1);

	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(10);		// Min 6 ns
}


/*
 * 	\brief Writes to ADS1299 register. Note that CS need to be active (LOW/RESET) during
 * 	the entire transmission!
 * 	@param address Address to register, use @ADS_REGISTERS
 * 	@param val Set value to register
 *
 */
void ADS_WriteReg(uint8_t address, uint8_t val)
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);

	uint8_t opcode = ADS_CMD_WREG + address;

	//	Stop Read Data Continuously
	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);



//	HAL_Delay(500);

	ADS_Transmit(&opcode, 1);

//	HAL_Delay(500);

	//	Write to 1 register (Ex. write to N registers => N-1)
	ADS_Transmit(&(uint8_t){0x00}, 1);

//	HAL_Delay(500);

	//	Set register to value
	ADS_Transmit(&val, 1);

	HAL_Delay(1);



//	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);	// denna kanske löser skiten!??!?!?

//	ADS_STOP();

	HAL_Delay(1); // TEST
}




//System Commands
void ADS_WAKEUP()
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
    ADS_Transmit(&(uint8_t){ADS_CMD_WAKEUP}, 1);
//    HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 4 T_CLK _CS high before new command

}

void ADS_STANDBY()			// only allowed to send WAKEUP after sending STANDBY
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
	ADS_Transmit(&(uint8_t){ADS_CMD_STANDBY}, 1);
//    HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 2 T_CLK?
}

void ADS_RESET()
{			// reset all the registers to default settings
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
    ADS_Transmit(&(uint8_t){ADS_CMD_RESET}, 1);
    HAL_Delay(1);		// Min 18 T_CLK for command to complete init
//    HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
    HAL_Delay(1);		// Min 2_TCLK?
}

void ADS_START()
{			//start data conversion
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
	ADS_Transmit(&(uint8_t){ADS_CMD_START}, 1);
//    HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 2 T_CLK?
}

void ADS_STOP()
{			//stop data conversion
//    HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
	ADS_Transmit(&(uint8_t){ADS_CMD_STOP}, 1);
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 2 T_CLK?

}

void ADS_RDATAC()
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
	ADS_Transmit(&(uint8_t){ADS_CMD_RDATAC}, 1);
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 4 T_CLK
}
void ADS_SDATAC()
{
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(1);		// Min 6 ns
	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);		// Min 4 T_CLK?
}

/*
 *	\brief Is used to initiate channel with settings.
 *	If not initiated, CHnSET will be iniated as 0x61.
 *	@param CHN_ADDR Channel address
 *	@ param SETTINGS use @CHnSET macros<. For example
 *	(.. , ..POWER_ON |  .._GAIN_4 | ... );
 *	Register bits not set, will be initiated as 0.
 */
void ADS_CHANNEL(uint8_t CHN_ADDR, uint8_t SETTINGS)
{
	if(SETTINGS & (ADS1299_CHN_POWER_OFF))
	{
		SETTINGS |= ADS1299_CHN_INPUT_INPUT_SHORTED;
	}
	ADS_WriteReg(CHN_ADDR, SETTINGS);
}

/*
 * 	\brief Is used to set the two registers BIAS_SENSP and BIAS_SENSN to desired value.
 * 	Route channel X to neg/pos signal to BIAS derivation 0 : Disabled | 1 : Enabled
 * 	Both registers, 0Dh and 0Eh (_SENSP and _SENSN) have default value 00h
 * 	@param SENSN BIAS_SENSP BIASNn bit mask definitions or leave blank
 * 	@param SENSN use BIAS_SENSP BIASPn bit mask definitions or leave blank
 * 	NOTE! This overwrites all previous values! (?) TODO: Does it though?
 */
void ADS_BIAS_SENS(uint8_t SENSN, uint8_t SENSP)
{
	ADS_WriteReg(ADS_BIAS_SENSN_ADDR, SENSN);
	ADS_WriteReg(ADS_BIAS_SENSP_ADDR, SENSP);
}

/*
 * 	\brief Is used to initiate CONFIG3 register
 * 	@param bitmask Use @CONFIG4 macros. Reserved values
 * 	will automatically be set. Other bits will be initiated
 * 	at 0 if not specified in bitmask
 * 	Default value 0x60
 */
void ADS_CONFIG4(uint8_t bitmask)
{
	bitmask &= ~(0b11110101);	// All but CONFIG4[3] & [1] are reserved to 0. This is used to ensure no mistake is made in the bitmask.
	ADS_WriteReg(ADS_CONFIG4_ADDR, bitmask);
}

/*
 * 	\brief Is used to initiate CONFIG3 register
 * 	@param bitmask Use @CONFIG3 macros. Reserved values
 * 	will automatically be set. Other bits will be initiated
 * 	at 0 if not specified in bitmask
 * 	Default value 0x60
 */
void ADS_CONFIG3(uint8_t bitmask)
{
	bitmask |= ADS1299_CONFIG3_RESERVED;
	ADS_WriteReg(ADS_CONFIG3_ADDR, bitmask);
	HAL_Delay(100);	//	Wait for internal reference to settle. TODO: Find out how much time that takes
}

/*
 * 	\brief Is used to initiate CONFIG2 register
 * 	@param bitmask Use @CONFIG2 macros. Reserved values
 * 	will automatically be set. Other bits  will be initiated
 * 	at 0 if not specified in bitmask..
 * 	Default value, 0xC0
 */
void ADS_CONFIG2(uint8_t bitmask)
{
	bitmask &= ~(0b101<<3);
	bitmask |= ADS1299_CONFIG2_RESERVED;
	ADS_WriteReg(ADS_CONFIG2_ADDR, bitmask);
}

/*
 * 	\brief Is used to initiate CONFIG1 register
 * 	@param bitmask Use @CONFIG1 macros. Reserved values
 * 	will automatically be set. Other bits  will be initiated
 * 	at 0 if not specified in bitmask..
 * 	Default value, 0x96
 */
void ADS_CONFIG1(uint8_t bitmask)
{
	bitmask &= ~(0b1<<3);
	bitmask |= ADS1299_CONFIG1_RESERVED;
//	if(bitmask && 0b111 == 0b111) // TODO: Fix stuff like this?
	ADS_WriteReg(ADS_CONFIG1_ADDR, bitmask);
}

/*
 * 	\brief Reading data from ADS. I.E ADS Data Output.
 * 	Sends 216 clock pulses and reads..
 * 	24 status bits
 * 	8*24 bits of data (24 bits for each 8 channels)
 * 	All data is placed in the DOUT struct. The data in each channel is presented with MSB first, in 2's complement.
 */
void ADS_DOUT()
{		// TODO: Reduce this delay!

	static uint32_t cCounter = 0;
	cCounter++;


	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	uint8_t temp[27];	// Temporary storage
	ADS_DRDY_Wait();	// Wait for DRDY to go active
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	for(int i = 0; i < 27; i++)
	{
		HAL_SPI_TransmitReceive(&ADS_SPI, &(uint8_t){0x00}, &temp[i], 1, 100);
		//HAL_SPI_Receive(&ADS_SPI, pData, Size, Timeout)
	}
//	HAL_SPI_TransmitReceive(&ADS_SPI, (uint8_t[27]){0}, temp, 27, 100);	// Receive data by sending 216 SCLKs

	// Handle data
	for(int i = 0; i < 3; i++)
	{
		DOUT.Status[i] = temp[i];
		DOUT.Channel_1[i] = temp[3*1+i];
		DOUT.Channel_2[i] = temp[3*2+i];
		DOUT.Channel_3[i] = temp[3*3+i];
		DOUT.Channel_4[i] = temp[3*4+i];
		DOUT.Channel_5[i] = temp[3*5+i];
		DOUT.Channel_6[i] = temp[3*6+i];
		DOUT.Channel_7[i] = temp[3*7+i];
		DOUT.Channel_8[i] = temp[3*8+i];
	}
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_Delay(1);
	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(1);
}

void ADS_Send()
{
	send_uart("Channel 1: ", huart2);
	HAL_UART_Transmit(&huart2, DOUT.Channel_1, 3, 100);
	send_uart("\n", huart2);
}

/*
 *
 */
void ADS_Plot()
{
	uint8_t tmp[3];
//	static uint32_t cCounter = 0;
//	cCounter++;
//	if(cCounter == 2000){cCounter = 0;}
	for(int i = 0; i<3; i++)
	{
		tmp[i] = DOUT.Channel_1[i];
		//if(tmp[i]==0xff){tmp[i]=0xfe;}
	}
//	tmp[0] = cCounter;
//	tmp[1] = cCounter>>8;
//	tmp[2] = cCounter>>16;
	HAL_UART_Transmit(&huart2, (uint8_t[]){0xff, 0xff}, 2, 100); // Start markers
//	HAL_UART_Transmit(&huart2, DOUT.Channel_1, 3, 100);
	HAL_UART_Transmit(&huart2, tmp, 3, 100);
//	HAL_UART_Transmit(&huart2, (uint8_t[]){'\n'}, 1, 100);
	HAL_UART_Transmit(&huart2, (uint8_t[]){'\r'}, 1, 100);

//	send_uart("\n", huart2);	// Only for looking at the data in YAT. Should not be used for plotting in python.
}

/*
 * 	\brief Should be called every 1/(2 kHz)
 * 	This can also control other
 */
void ADS_DRDY_GPIO_EXTI(uint16_t ADS_N_DRDY_PIN)
{
	if(ADS_N_DRDY_PIN == ADS_DRDY_PIN){
		ADS_DOUT();
	    ADS_Plot();
	}
}
