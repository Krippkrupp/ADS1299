/*
 * ADS1299_drivers.c
 *
 *  Created on: Jul 12, 2021
 *      Author: westr
 */
#include "ADS1299_drivers.h"
#include "main.h"

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
	HAL_SPI_Transmit(&hspi3, (uint8_t *) data, size, 100);
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);



	HAL_Delay(10);
//	hodl();
}


/*
 * 	\brief Is used to wait for DRDY to go low before sending data
 */
void ADS_DRDY_Wait(){
	while(HAL_GPIO_ReadPin(ADS_DRDY_BUS, ADS_DRDY_PIN) == GPIO_PIN_SET);
	HAL_Delay(1); // TODO: is this needed?
}


void ADS_device_init()
{
	/*****************TEMPORÄR TA BORT FRÅN HÄR***********************/
//	penis:
//	ADS_Transmit(&(uint8_t){0x96}, 1);
//	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == RESET);
//	HAL_Delay(500);
//	goto penis;

	/*****************TEMPORÄR TA BORT TILL HIT***********************/
//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
//	HAL_Delay(90000);
	//if possible clear PWDN
	// wait 1 ms
	//if possible set PWDN

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
	while(1)
	{
		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);

//		HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
//		HAL_Delay(100);
		hodl();



//		for(int i = 0; i<27; i++)
//		{

			ADS_DRDY_Wait();
//			ADS_Transmit(&(uint8_t){0x00}, 27);
			HAL_SPI_Receive(&hspi3, ads_data, 27, 100);

			HAL_UART_Transmit(&huart2, ads_data, 27, 100);
			// ONSDAG BÖRJA HÄR
	}



	send_uart("delaying..\n", huart2);
	HAL_Delay(900000);

}

void ADS_test()
{
	HAL_Delay(500);

	ADS_Transmit(&(uint8_t) {ADS_CMD_RESET}, 1);



	HAL_Delay(500);	// Wait until power goes up completely (minimum 180ms)

//	HAL_Delay(129);

	//	ADS is waken up in RDATAC mode

//	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC},1);

//	ADS_Transmit(ADS_CMD_STOP, 1);

//	ADS_WriteReg(ADS_CONFIG3_ADDR, 0xE0);

	ADS_ReadReg(ADS_CONFIG1_ADDR, 14);

//	ADS_WriteReg(ADS_CONFIG1_ADDR, 0x60);



//	uint8_t temp;
//	for(int i = 0; i < 20; i++)
//	{
//		ADS_DRDY_Wait();
//		HAL_SPI_Receive(&hspi3, &temp, 1, 100);
//		HAL_Delay(1);
//	}
	send_uart("jag kom hit\n", huart2);
	HAL_Delay(900000);

}



void ADS_PowerOn()
{
	// Wait for power-up etc.
	HAL_Delay(200);

	//	Issue reset pulse & wait for 18 T_CLK

	ADS_Transmit(&(uint8_t) {ADS_CMD_RESET}, 1);



	HAL_Delay(500);	// Wait until power goes up completely (minimum 180ms)

	//	ADS is waken up in RDATAC mode

	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC},1);

	// Using Internal Reference
	ADS_WriteReg(ADS_CONFIG3_ADDR, 0xE0U);


	ADS_WriteReg(ADS_CONFIG1_ADDR, 0x96U);

	ADS_WriteReg(ADS_CONFIG2_ADDR, 0xC0U);

	ADS_WriteReg(ADS_CH1SET_ADDR, 0x01);

	ADS_WriteReg(ADS_CH2SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH3SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH4SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH5SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH6SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH7SET_ADDR, 0x81U);
	ADS_WriteReg(ADS_CH8SET_ADDR, 0x81U);

	ADS_Transmit(&(uint8_t){ADS_CMD_START}, 1);

	ADS_Transmit(&(uint8_t){ADS_CMD_RDATAC}, 1);

//	ADS_ReadReg(ADS_CONFIG1_ADDR, 0x17);

	uint8_t temp;
	ADS_ReadReg(ADS_CONFIG1_ADDR, 17);
	HAL_Delay(2000);
	for(int i = 0; i < 20; i++)
	{
		ADS_DRDY_Wait();
		HAL_SPI_Receive(&hspi3, &temp, 1, 100);
		HAL_Delay(200);
	}




	uint8_t tmp[3];


	 while (1)
	  {
		 ADS_DRDY_Wait();	// Waits for data ready
		 HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
		 HAL_Delay(1);
		 HAL_SPI_Receive(&hspi3, tmp, 3, 100);
		 HAL_SPI_Receive(&hspi3, tmp, 3, 100);
		 HAL_SPI_Receive(&hspi3, tmp, 3, 100);
		 HAL_SPI_Receive(&hspi3, tmp, 3, 100);
		 HAL_Delay(1);
		 HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
		 //		 ADS_Transmit(&(uint8_t){0}, 3);
//		  ADS_Transmit(&(uint8_t){0}, 3);
		  HAL_Delay(200);

	  }


	HAL_Delay(9999999);
}

//void ADS_ChannelSet(uint8_t channel, uint8_t PowerDown)
//{
//	ADS_Transmit(&(uint8_t){(ADS_CMD_WREG | channel)}, 1);
//
//}


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


	// test
//	ADS_DRDY_Wait();
//	uint8_t temp[numOfReg];
//	HAL_SPI_Receive(&hspi3, temp, numOfReg, 100);

	//end test

	//	TODO: Ta bort när denna är helt ok!
	for(int i = 0; i < numOfReg; i++)
	{
		ADS_DRDY_Wait();
		ADS_Transmit(&(uint8_t){0x00}, 1);
		HAL_Delay(50);
	}

//	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
//	HAL_Delay(1);		// Min 6 ns

}

// Remove? A bit redundant
void ADS_RReg(uint8_t baseAddr)
{
	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, RESET);
	HAL_Delay(200);		// Min 6 ns

	uint8_t opcode = (ADS_CMD_RREG | baseAddr);
	ADS_Transmit(&opcode, 1);

	// Read one register
	ADS_Transmit(&(uint8_t){0x00}, 1);

	//	Receive
	ADS_DRDY_Wait();
	ADS_Transmit(&(uint8_t){0x00}, 1);

	HAL_GPIO_WritePin(ADS_CS_BUS, ADS_CS_PIN, SET);
	HAL_Delay(200);		// Min 6 ns
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

	HAL_Delay(10);



//	ADS_Transmit(&(uint8_t){ADS_CMD_SDATAC}, 1);	// denna kanske löser skiten!??!?!?

//	ADS_STOP();

	HAL_Delay(10); // TEST
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
	ADS_WriteReg(CHN_ADDR, SETTINGS);
}

