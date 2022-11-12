/*
 *   \file ADS1299_driver.h
 *  \brief Drivers for Texas Instruments ADS1299 with register and timing definitions, functions for different settings in which to run the ADS.
 * 	The library was written for Department of Biomedicine at Lunds University.
 *	\author Kristoffer Westring
 *	\version 1.0
 *	\date July 2021
 */

/*
 * 	Information:
 * 	- SPI mode 1; CPOL=0, CPHA=1 are the correct settings. However for the board that I used, SPI mode 2; CPOL=1, CPHA=0 works.
 *
 * 	~ Things to add
 * 	+	Support for multiple ADS
 * 	+	Error handling (Almost none in place as of now)
 * 	+	Add functions to calculate the conversions
 * 	+
 *
 */


#ifndef INC_ADS1299_DRIVERS_H_
#define INC_ADS1299_DRIVERS_H_

#include "main.h"


/*
 * 	User defined macros
 */
#define		ADS_BAUDRATE		2.34375			/*!<	Needs to be entered by user depending on SPI settings. Unit: Mbit/s 	*/
#define 	ADS_MASTER_CLK		F_CLK			/*!<	TODO: Is this correct? (No? Master CLK is probably CLK. Maybe remove this one, it may be redundant) Unit: MHz			*/
#define 	ADS_CS_BUS			GPIOD			/*!<	Chip Select (CS) bus for ADS1299. Needs to be configured according to user setup.	*/
#define		ADS_CS_PIN			GPIO_PIN_6		/*!<	Chip Select (CS) pin for ADS1299. Needs to be configured according to user setup.	*/
#define		ADS_DRDY_BUS		GPIOD			/*!<	Data Ready (DRDY) bus for ADS1299. Needs to be configured according to user setup.		*/
#define		ADS_DRDY_PIN		GPIO_PIN_3		/*!<	Data Ready (DRDY) pin for ADS1299. Needs to be configured according to user setup.	*/
#define		ADS_START_BUS		GPIOD			/*!<	START bus for ADS1299. Needs to be configured according to user setup.	*/
#define		ADS_START_PIN		GPIO_PIN_7		/*!<	START pin for ADS1299. Needs to be configured according to user setup.	*/
#define 	ADS_SPI				hspi3			/*!<	SPI peripheral for ADS1299. Needs to be configured according to user setup.	*/

#define 	BUFFER_SIZE			100				/*!<  	Buffer plot size, used to plot in python	*/
/*
 * 	@ADS_TIME
 * 	Timing requirement macros. Please note, these are defined assuming 2.7 V <= DVDD <= 3.6 V. TODO: Add macros for other DVDD voltages
 */
#define		F_CLK			2.048			/*!<	Clock frequency. Internal f_clk nominal 2.048 MHz. Unit: MHz	*/
#define		T_CLK			489		/*!<	Master clock period. Needs to be entered according to internal/external clk. 1/f_clk. Unit: ns. 414ns<=T_CK<=666		*/
#define		T_SCLK			50		/*!<	SCLK period. Min 50 ns. Unit: ns		*/
#define		T_SPWH			15			/*!<	Pulse duration, SCLK pulse duration, high or low. Min 15 ns. Unit: ns		*/
#define		T_DIST			10			/*!<	Setup time, DIN valid to SCLK falling edge . Min 10 ns. Unit: ns		*/
#define		T_DIHD			10			/*!<	Hold time, valid DIN after SCLK falling edge. Min 10 ns. Unit: ns		*/
#define		T_CSH			2*T_CLK			/*!<	Pulse duration, N_CS high. Min 2*T_CLK. Unit T_CLK.		*/
#define		T_SCCS			4*T_CLK			/*!<	Delay time, final SCLK falling edge to N_CS high. Min 4*T_CLK. Unit T_CLK. 		*/
#define		T_SDECODE		4*T_CLK				/*!<	Command decode time. Min 4*T_CLK. Unit T_CLK.		*/
#define		T_DISCK2ST		10				/*!<	Setup time, DAISY_IN valid to SCLK rising edge. Min 10 ns. Unit: ns		*/
#define		T_DISCK2HT		10				/*!<	Hold time, DAISY_IN valid after SCLK rising edge. Min 10 ns. Unit: ns		*/



/*
 * 	@ADS_SWITCH
 * 	Switching Characteristics macros for Serial Interface
 */
#define		T_DOHD			10			/*!<	Hold time, SCLK falling edge to invalid DOUT. Min 10 ns. Unit: ns		*/
#define		T_DOPD			17			/*!<	Propagation delay time, SCLK rising edge to DOUT valid. Max 17 ns. Unit: ns		*/
#define		T_CSDOD			10			/*!<	Propagation delay time, N_CS low to DOUT driven. Min 10 ns. Unit: ns 		*/
#define		T_CSDOZ			10			/*!<	Propagation delay time, N_CS high to DOUT Hi-Z. Max 10 ns. Unit: ns		*/


#define 	T_POR		262145*T_CLK		/*!<		*/
#define 	T_RST		2*T_CLK				/*!<		*/

/*
 * 	@ADS_COMMANDS
 * 	Commands for Serial Interface on ADS1299
 */
		//	System Commands
#define		ADS_CMD_WAKEUP		0x02U			/*!<	Wake-up from standby mode		*/
#define		ADS_CMD_STANDBY		0x04U			/*!<	Enter standby mode		*/
#define		ADS_CMD_RESET		0x06U			/*!<	Reset the device		*/
#define		ADS_CMD_START		0x08U			/*!<	Start and restart (synchronize) conversions		*/
#define		ADS_CMD_STOP		0x0AU			/*!<	Stop conversion		*/
		//	Data Read Commands
#define		ADS_CMD_RDATAC		0x10U			/*!<	Enable Read Data Continuous mode. This mode is the default mode at power-up.		*/
#define		ADS_CMD_SDATAC		0x11U			/*!<	Stop Read Data Continuously mode		*/
#define		ADS_CMD_RDATA		0x12U			/*!<	Read data by command; supports multiple read back		*/
		//	Register Read Commands TODO: Any nice way to solve these two?
#define		ADS_CMD_RREG		0x20U			/*!<	Read n nnnn registers starting at address r rrrr		*/
#define		ADS_CMD_WREG		0x40U			/*!<	Write n nnnn registers starting at address r rrrr		*/


/*
 * 	@ADS_REGISTERS
 * 	Register Map for ADS1299. TODO: Check that they work
 */
	//	Read Only ID Registers
#define			ADS_ID_ADDR				0x00U			/*!<			*/
	//	Global Settings Across Channels
#define			ADS_CONFIG1_ADDR		0x01U			/*!<			*/
#define			ADS_CONFIG2_ADDR		0x02U			/*!<			*/
#define			ADS_CONFIG3_ADDR		0x03U			/*!<			*/
#define			ADS_LOFF_ADDR			0x04U			/*!<			*/
	//	Channel-Specific Settings
#define			ADS_CH1SET_ADDR			0x05U			/*!<			*/
#define			ADS_CH2SET_ADDR			0x06U			/*!<			*/
#define			ADS_CH3SET_ADDR			0x07U			/*!<			*/
#define			ADS_CH4SET_ADDR			0x08U			/*!<			*/
#define			ADS_CH5SET_ADDR			0x09U			/*!<			*/
#define			ADS_CH6SET_ADDR			0x0AU			/*!<			*/
#define			ADS_CH7SET_ADDR			0x0BU			/*!<			*/
#define			ADS_CH8SET_ADDR			0x0CU			/*!<			*/
#define			ADS_BIAS_SENSP_ADDR		0x0DU			/*!<			*/
#define			ADS_BIAS_SENSN_ADDR		0x0EU			/*!<			*/
#define			ADS_LOFF_SENSP_ADDR		0x0FU			/*!<			*/
#define			ADS_LOFF_SENSN_ADDR		0x10U			/*!<			*/
#define			ADS_LOFF_FLIP_ADDR		0x11U			/*!<			*/
	//	Lead-Off Status Registers (Read-Only Registers)
#define			ADS_LOFF_STATP_ADDR		0x12U			/*!<			*/
#define			ADS_LOFF_STATN_ADDR		0x13U			/*!<			*/
	//	GPIO and OTHER Registers
#define			ADS_GPIO_ADDR			0x14U			/*!<			*/
#define			ADS_MISC1_ADDR			0x15U			/*!<			*/
#define			ADS_MISC2_ADDR			0x16U			/*!<			*/
#define			ADS_CONFIG4_ADDR		0x17U			/*!<			*/


/*************	BIT MASK DEFINITIONS	********************/

/********************************************
 *				@CONFIG1					*
 *	Configures daisy, clock and data rate	*
 ********************************************/
#define		ADS1298_CONFIG1_RESET		0x96U		/*!<	Reset value for CONFIG1 register.	*/
#define		ADS1299_CONFIG1_RESERVED	(0b1001<<4)	/*!<	Reserved values for CONFIG1 register. Not that CONFIG1[3] is reserved as 0	*/

/**
 *  \brief CONFIG1 DAISY_N bit mask definitions
 *	Daisy-chain of multiple readback mode.
 *	Default value, 0h, _DAISY_ENABLE
 */
#define		ADS1299_CONFIG1_DAISY_ENABLE		(0<<6)		/*!<	Daisy-chain mode	*/
#define		ADS1299_CONFIG1_DAISY_DISABLE		(1<<6)		/*!<	Multiple readback mode	*/

/**
 *  \brief CONFIG1 CLK_EN bit mask definitions
 *  Determines if the internal oscillator signal is connected to CLK pin, when the CLKSEL pin = 1.
 *	Default value, 0h, _DISABLE
 */
#define		ADS1299_CONFIG1_CLOCK_ENABLE		(1<<5)		/*!<	Oscillator clock output enabled.	*/
#define		ADS1299_CONFIG1_CLOCK_DISABLE		(0<<5)		/*!<	Oscillator clock output disabled.	*/

/**
 *  \brief CONFIG1 DR bit mask definitions
 *  F_MOD = F_CLK/2. F_CLK is clock frequency. Internal clock is 2.048 MHz
 *	Default value, 6h, _FMOD_DIV_4096
 */
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_64		0x00U		/*!<	Determines output data date of device. 16 kSPS @ 2.048 MHz. F_MOD/64.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_128		0x01U		/*!<	Determines output data date of device. 8 kSPS @ 2.048 MHz. F_MOD/128.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_256		0x02U		/*!<	Determines output data date of device. 4 kSPS @ 2.048 MHz. F_MOD/256.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_512		0x03U		/*!<	Determines output data date of device. 2 kSPS @ 2.048 MHz. F_MOD/512.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_1024		0x04U		/*!<	Determines output data date of device. 1 kSPS @ 2.048 MHz. F_MOD/1024.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_2048		0x05U		/*!<	Determines output data date of device. 500 SPS @ 2.048 MHz. F_MOD/2048.	*/
#define		ADS1299_CONFIG1_DATA_RATE_FMOD_DIV_4096		0x06U		/*!<	Determines output data date of device. 250 SPS @ 2.048 MHz. F_MOD/4096.	*/


/********************************************************
 *						@CONFIG2		    				*
 * This register configures the test signal generation  *
 ********************************************************/
#define		ADS1298_CONFIG2_RESET		0xC0U		/*!<	Reset value for CONFIG2 register.	*/
#define		ADS1299_CONFIG2_RESERVED 	(0b11<<6)	/*!<	Reserved values for CONFIG3 register. Note that CONFIG[5] & [3] is reserved to 0.	*/
/**
 *  \brief CONFIG2 INT_CAL bit mask definitions
 *  Determines the source for the test signals.
 *	Default value, 0h, _CAL_EXT
 */
#define		ADS1299_CONFIG2_CAL_EXT		(0<<4)		/*!<	Test signals are driven externally	*/
#define		ADS1299_CONFIG2_CAL_INT		(1<<4)		/*!<	Test signals are driven internally	*/


/**
 *  \brief CONFIG2 CAL_AMP bit mask definitions
 *  Determines the amplitude for the calibration signals.
 *	Default value, 0h, _AMP_VREF_DIV_2400
 */
#define		ADS1299_CONFIG2_CAL_AMP_VREF_DIV_2400		(0<<2)		/*!<	1 × –(VREFP – VREFN) / 2400	*/
#define		ADS1299_CONFIG2_CAL_AMP_2_VREF_DIV_2400		(1<<2)		/*!<	2 × –(VREFP – VREFN) / 2400	*/

/**
 *  \brief CONFIG2 CAL_FREQ bit mask definitions
 *  Determines the frequency for the calibration signals.
 *	Default value, 0h, _FCLK_DIV_2_21
 */
#define		ADS1299_CONFIG2_CAL_FREQ_FCLK_DIV_2_21		0		/*!<	Pulsed at F_CLK/(2^21), ~1 Hz @ 2.048 MHz	*/
#define		ADS1299_CONFIG2_CAL_FREQ_FCLK_DIV_2_20		1		/*!<	Pulsed at F_CLK/(2^20), ~2 Hz @ 2.048 MHz	*/
#define		ADS1299_CONFIG2_CAL_FREQ_DC					3		/*!<	At DC. Not pulsed.	*/


/********************************************************
 *						@CONFIG3		    				*
 * Configures int. or ext. reference and BIAS operation *
 ********************************************************/
#define		ADS1299_CONFIG3_RESET		0x60U		/*!<	Reset value for CONFIG3 register.	*/
#define		ADS1299_CONFIG3_RESERVED 	(0b11<<5)	/*!<	Reserved values for CONFIG3 register. Needs to be set	*/
/**
 *  \brief CONFIG3 PD_REFBUF bit mask definitions.
 *	Controls if the internal voltage reference buffer should be enabled or disabled
 *	Disabling REFBUF requires external reference on VREFP in order to operate.
 *	Default value, 0h, _DISABLED
 */
#define		ADS1299_CONFIG3_INT_REF_BUF_DISABLE		(0<<7)		/*!<	Power-down internal reference buffer	*/
#define		ADS1299_CONFIG3_INT_REF_BUF_ENABLE		(1<<7)		/*!<	Enable internal reference buffeR	*/

/**
 *  \brief CONFIG3 BIAS_MEAS bit mask definitions.
 * 	Enables or disables BIAS measurement
 *	Default value, 0h, _DISABLE
 */
#define		ADS1299_CONFIG3_BIAS_MEAS_DISABLE		(0<<4)		/*!<	Open	*/
#define		ADS1299_CONFIG3_BIAS_MEAS_ENABLE		(1<<4)		/*!<	BIAS_IN signal is routed to the channel that has the MUX_Setting 010	*/

/**
 *  \brief CONFIG3 BIASREF_INT bit mask definitions.
 * 	Determines BIASREF signal source.
 *	Default value 0, _BIASREF_EXT
 */
#define		ADS1299_CONFIG3_BIASREF_EXT		(0<<3)		/*!<	External BIAS reference	*/
#define		ADS1299_CONFIG3_BIASREF_INT		(1<<3)		/*!<	Internal BIAS reference, (AVDD+AVSS)/2 generated internally	*/

/**
 *  \brief CONFIG3 PD_BIAS bit mask definitions.
 * 	Determines BIAS buffer power state, i.e power-down or enable bias buffer amplifier
 *	Default value, 0h, _DISABLE
 */
#define		ADS1299_CONFIG3_BIAS_BUFFER_DISABLE		(0<<2)		/*!<	BIAS buffer is disabled	*/
#define		ADS1299_CONFIG3_BIAS_BUFFER_ENABLE		(1<<2)		/*!<	BIAS buffer is enabled	*/

/**
 *  \brief CONFIG3 BIAS_LOFF_SENSE bit mask definitions.
 * 	Determines BIAS sense function to be enabled or disabled.
 *	Default value, 0h, _DISABLE
 */
#define		ADS1299_CONFIG3_BIAS_LOFF_SENSE_DISABLE		(0<<1)		/*!<	BIAS sense is disabled	*/
#define		ADS1299_CONFIG3_BIAS_LOFF_SENSE_ENABLE		(1<<1)		/*!<	BIAS sense is enabled	*/

/**
 *  \brief CONFIG3 PD_BIAS bit mask definitions.
 * 	Determines BIAS lead-off status
 *	Default value, 0h, _CONNECTED
 */
#define		ADS1299_CONFIG3_BIAS_STAT_CONNECTED			0		/*!<	BIAS is	connected */
#define		ADS1299_CONFIG3_BIAS_STAT_NOT_CONNECTED		1		/*!<	BIAS is	not connected */


/********************************************************
 *						LOFF		    				*
 *      Configures the lead-off detection operation.    *
 ********************************************************/
 #define		ADS1298_LOFF_RESET	0x00U		/*!<	Reset value for LOFF register.	*/
 /**
 *  \brief LOFF COMP_TH bit mask definitions (lead-off comparator threshold)
 *
 *	The definitions are with respect to the positive side, LOFFP.
 *	The corresponding thresholds for LOFFN is the 100% minus LOFFP percent value.
 *	Default value, 0b000<<5, _95_PERCENT.
 */
#define		ADS1299_LOFF_COMP_TH_95_PERCENT			(0b000<<5)						/*!<	95 percent based of comparator positive side. Negative side will be 100-95=5%		*/
#define		ADS1299_LOFF_COMP_TH_92_5_PERCENT		(0b001<<5)						/*!<	92.5 percent based of comparator positive side. Negative side will be 100-92.5=7.5%		*/
#define		ADS1299_LOFF_COMP_TH_90_PERCENT			(0b010<<5)						/*!<	90 percent based of comparator positive side. Negative side will be 100-90=10%		*/
#define		ADS1299_LOFF_COMP_TH_87_5_PERCENT		(0b011<<5)						/*!<	87.5 percent based of comparator positive side. Negative side will be 100-87.5=12.5%		*/
#define		ADS1299_LOFF_COMP_TH_85_PERCENT			(0b100<<5)						/*!<	85 percent based of comparator positive side. Negative side will be 100-85=15%		*/
#define		ADS1299_LOFF_COMP_TH_80_PERCENT			(0b101<<5)						/*!<	80 percent based of comparator positive side. Negative side will be 100-80=20%		*/
#define		ADS1299_LOFF_COMP_TH_75_PERCENT			(0b110<<5)						/*!<	75 percent based of comparator positive side. Negative side will be 100-75=25%		*/
#define		ADS1299_LOFF_COMP_TH_70_PERCENT			(0b111<<5)						/*!<	70 percent based of comparator positive side. Negative side will be 100-70=30%		*/

 /**
 *  \brief LOFF ILEAD_OFF bit mask definitions (Lead-off current magnitude)
 *
 *	These bits determine the magnitude of current for the current lead-off mode.
 *	Default value, 0b00<<2, _6_NANOAMP
 */
#define		ADS1299_LOFF_ILEAD_OFF_6_NANOAMP		(0b00<<2)					/*!<	6 nA lead-off current		*/
#define		ADS1299_LOFF_ILEAD_OFF_24_NANOAMP		(0b01<<2)					/*!<	24 nA lead-off current		*/
#define		ADS1299_LOFF_ILEAD_OFF_6_MICROAMP		(0b10<<2)					/*!<	6 µA lead-off current		*/
#define		ADS1299_LOFF_ILEAD_OFF_24_MICROAMP		(0b11<<2)					/*!<	24 µA lead-off current		*/

 /**
 *  \brief LOFF FLEAD_OFF bit mask definitions (Lead-off frequency)
 *
 *	These bits determine the magnitude of current for the current lead-off mode.
 *	Default value, 0b00<<2, _6_NANOAMP
 */
#define		ADS1299_LOFF_FLEAD_OFF_DC				(0b00)		/*!<	DC lead-off detection		*/
#define		ADS1299_LOFF_FLEAD_OFF_FCLK_DIV_2_18	(0b01)		/*!<	AC lead-off detection at F_CLK/(2^(18)). 7.8 Hz @ 2.048 Mhz		*/
#define		ADS1299_LOFF_FLEAD_OFF_FCLK_DIV_2_16	(0b10)		/*!<	AC lead-off detection at F_CLK/(2^(16)). 31.2 Hz @ 2.048 Mhz		*/
#define		ADS1299_LOFF_FLEAD_OFF_FDR_DIV_4		(0b11)		/*!<	AC lead-off detection at F_DR/4		*/



/********************************************
 *				@CHNnSET						*
 * Individual Channel Settings	 (n=1 to 8)	*
 ********************************************/
#define		ADS1299_CHNNSET_RESET		0x61U		/*!<	Reset value	for CHnSET register */

/**
 *  \brief CHNnSET PD bit mask definition.
 *	Controls the channel power.
 *	See ADS1299 (rev. SBAS499C) datasheet, p. 50 , for information.
 */
#define 	ADS1299_CHN_POWER_ON		(0<<7)			/*!<	Channel turned on.	*/
#define 	ADS1299_CHN_POWER_OFF		(1<<7)			/*!<	Channel turned off. Recommended to set MUX to input shorted.	*/


/**
 *  \brief CHNnSET GAIN bit mask definition.
 *	Controls the programmable gain amplifier (PGA).
 * See ADS1299 (rev. SBAS499C) datasheet, p. 50 , for information.
 */
#define 	ADS1299_CHN_GAIN_1			(0<<4)		/*!<	Channel gain 1	*/
#define 	ADS1299_CHN_GAIN_2			(1<<4)		/*!<	Channel gain 2	*/
#define 	ADS1299_CHN_GAIN_4			(2<<4)		/*!<	Channel gain 4	*/
#define 	ADS1299_CHN_GAIN_6			(3<<4)		/*!<	Channel gain 6	*/
#define 	ADS1299_CHN_GAIN_8			(4<<4)		/*!<	Channel gain 8	*/
#define 	ADS1299_CHN_GAIN_12			(5<<4)		/*!<	Channel gain 12	*/
#define 	ADS1299_CHN_GAIN_24			(6<<4)		/*!<	Channel gain 24	*/

/**
 *  \brief CHNnSET SRB22 bit mask definitions
 *  Default value 0<<3, _OPEN
 */
#define 		ADS1299_CHN_SRB2_OPEN		(0<<3)		/*!<		*/
#define 		ADS1299_CHN_SRB2_CLOSED		(1<<3)		/*!<		*/


/**
 *  \brief CHNnSET MUX bits, bit mask definition.
 *	Note that some channel settings may require crtain MUX settings.
 *	For example, if powering down channel, it's recommended to set input shorted.
 * See ADS1299 (rev. SBAS499C) datasheet, p. 20-21 and p. 50 , for information.
 */
#define 	ADS1299_CHN_INPUT_NORMAL_ELECTRODE			0x00U		/*!<	Channel connected to corresponding INxP and INxN input.	*/
#define 	ADS1299_CHN_INPUT_INPUT_SHORTED				0x01U		/*!<	Channel inputs are shorted, for offset or noise measurements. See ADS1299 datasheet p. 21 (rev. SBAS499C)	*/
#define 	ADS1299_CHN_INPUT_BIAS_MEASUREMENT			0x02U		/*!<	Used in conjunction with BIAS_MEAS bit in CONFIG3 for BIAS measurements. See ADS1299 datasheet p. 20 (rev. SBAS499C)	*/
#define 	ADS1299_CHN_INPUT_MVDD_SUPPLY				0x03U		/*!<	Used for measuring analog and digital supplies. See ADS1299 datasheet p. 21 (rev. SBAS499C)	*/
#define 	ADS1299_CHN_INPUT_TEMPERATURE_SENSOR		0x04U		/*!<	Measures temperature using the on-chip temperature sensors. See ADS1299 datasheet p. 21 (rev. SBAS499C)	*/
#define 	ADS1299_CHN_INPUT_TEST_SIGNAL				0x05U		/*!<	Measures the calibration signal. See ADS1299 datasheet p. 21 (rev. SBAS499C)	*/
#define 	ADS1299_CHN_INPUT_BIAS_DRIVE_P				0x06U		/*!<	Shorts INxP to bias drive output.	*/
#define 	ADS1299_CHN_INPUT_BIAS_DRIVE_N				0x07U		/*!<	Shorts INxN to bias drive output.	*/



/********************************************************
 *						BIAS_SENSP	    				*
 *      Configures selection of pos signals from each   *
 *		channel for bias voltage (BIAS) derivation.		*
 *		See p. 33 in ADS1299 datasheet (rev.SBAS499C)	*
 ********************************************************/
 #define		ADS1298_BIAS_SENSP_RESET	0x00U		/*!<	Reset value for BIAS_SENSP register.	*/

/**
 *  \brief BIAS_SENSP BIASPn bit mask definitions
 *	Use the bias circuitry to counter the common-mode interference
 *	in a EEG system as a result of power lines and
 *	other sources, including fluorescent lights.
 *	Default value, 0 (Disabled)
 */
#define		ADS1299_BIAS_SENSP_BIASP8_ENABLED		(1<<7)			/*!<	Enable route channel 8 positive (IN8P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP7_ENABLED		(1<<6)			/*!<	Enable route channel 7 positive (IN7P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP6_ENABLED		(1<<5)			/*!<	Enable route channel 6 positive (IN6P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP5_ENABLED		(1<<4)			/*!<	Enable route channel 5 positive (IN5P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP4_ENABLED		(1<<3)			/*!<	Enable route channel 4 positive (IN4P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP3_ENABLED		(1<<2)			/*!<	Enable route channel 3 positive (IN3P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP2_ENABLED		(1<<1)			/*!<	Enable route channel 2 positive (IN2P) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSP_BIASP1_ENABLED		(1<<0)			/*!<	Enable route channel 1 positive (IN1P) signal into BIAS derivation	*/


/********************************************************
 *						BIAS_SENSN	    				*
 *      Configures selection of neg signals from each   *
 *		channel for bias voltage (BIAS) derivation.		*
 *		See p. 33 in ADS1299 datasheet (rev.SBAS499C)	*
 ********************************************************/
 #define		ADS1298_BIAS_SENSN_RESET	0x00U		/*!<	Reset value for BIAS_SENSN register.	*/

/**
 *  \brief BIAS_SENSP BIASNn bit mask definitions
 *	Use the bias circuitry to counter the common-mode interference
 *	in a EEG system as a result of power lines and
 *	other sources, including fluorescent lights.
 *	Default value, 0 (Disabled)
 */
#define		ADS1299_BIAS_SENSN_BIASN8_ENABLED		(1<<7)			/*!<	Enable route channel 8 negative (IN8N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN7_ENABLED		(1<<6)			/*!<	Enable route channel 7 negative (IN7N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN6_ENABLED		(1<<5)			/*!<	Enable route channel 6 negative (IN6N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN5_ENABLED		(1<<4)			/*!<	Enable route channel 5 negative (IN5N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN4_ENABLED		(1<<3)			/*!<	Enable route channel 4 negative (IN4N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN3_ENABLED		(1<<2)			/*!<	Enable route channel 3 negative (IN3N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN2_ENABLED		(1<<1)			/*!<	Enable route channel 2 negative (IN2N) signal into BIAS derivation	*/
#define		ADS1299_BIAS_SENSN_BIASN1_ENABLED		(1<<0)			/*!<	Enable route channel 1 negative (IN1N) signal into BIAS derivation	*/



/********************************************************
 *						LOFF_SENSP	    				*
 *      Configures selection of pos signals from each   *
 *		channel for lead-off detection.					*
 *		See p. 30 in ADS1299 datasheet (rev.SBAS499C)	*
 ********************************************************/
#define		ADS1298_LOFF_SENSP_RESET	0x00U		/*!<	Reset value for LOFF_SENSP register.	*/

/**
 *  \brief LOFF_SENSP LOFFPn bit mask definitions
 *	Default value, 0 (Disabled)
 */
#define		ADS1299_LOFF_SENSP_LOFFP8_ENABLED		(1<<7)			/*!<	Enable lead-off detection on IN8P	*/
#define		ADS1299_LOFF_SENSP_LOFFP7_ENABLED		(1<<6)			/*!<	Enable lead-off detection on IN7P	*/
#define		ADS1299_LOFF_SENSP_LOFFP6_ENABLED		(1<<5)			/*!<	Enable lead-off detection on IN6P	*/
#define		ADS1299_LOFF_SENSP_LOFFP5_ENABLED		(1<<4)			/*!<	Enable lead-off detection on IN5P	*/
#define		ADS1299_LOFF_SENSP_LOFFP4_ENABLED		(1<<3)			/*!<	Enable lead-off detection on IN4P	*/
#define		ADS1299_LOFF_SENSP_LOFFP3_ENABLED		(1<<2)			/*!<	Enable lead-off detection on IN3P	*/
#define		ADS1299_LOFF_SENSP_LOFFP2_ENABLED		(1<<1)			/*!<	Enable lead-off detection on IN2P	*/
#define		ADS1299_LOFF_SENSP_LOFFP1_ENABLED		(1<<0)			/*!<	Enable lead-off detection on IN1P	*/



/********************************************************
 *						LOFF_SENSN	    				*
 *      Configures selection of neg signals from each   *
 *		channel for lead-off detection.					*
 *		See p. 30 in ADS1299 datasheet (rev.SBAS499C)	*
 ********************************************************/
#define		ADS1298_LOFF_SENSN_RESET	0x00U		/*!<	Reset value for LOFF_SENSN register.	*/

/**
 *  \brief LOFF_SENSN LOFFNn bit mask definitions
 *	Default value, 0 (Disabled)
 */
#define		ADS1299_LOFF_SENSN_LOFFN8_ENABLED		(1<<7)			/*!<	Enable lead-off detection on IN8N	*/
#define		ADS1299_LOFF_SENSN_LOFFN7_ENABLED		(1<<6)			/*!<	Enable lead-off detection on IN7N	*/
#define		ADS1299_LOFF_SENSN_LOFFN6_ENABLED		(1<<5)			/*!<	Enable lead-off detection on IN6N	*/
#define		ADS1299_LOFF_SENSN_LOFFN5_ENABLED		(1<<4)			/*!<	Enable lead-off detection on IN5N	*/
#define		ADS1299_LOFF_SENSN_LOFFN4_ENABLED		(1<<3)			/*!<	Enable lead-off detection on IN4N	*/
#define		ADS1299_LOFF_SENSN_LOFFN3_ENABLED		(1<<2)			/*!<	Enable lead-off detection on IN3N	*/
#define		ADS1299_LOFF_SENSN_LOFFN2_ENABLED		(1<<1)			/*!<	Enable lead-off detection on IN2N	*/
#define		ADS1299_LOFF_SENSN_LOFFN1_ENABLED		(1<<0)			/*!<	Enable lead-off detection on IN1N	*/


/********************************************************
 *						LOFF_FLIP	    				*
 *      Configures direction of current used for	    *
 *		lead-off derivation.							*
 *		See p. 30 in ADS1299 datasheet (rev.SBAS499C)	*
 ********************************************************/
#define		ADS1298_LOFF_FLIP_RESET	0x00U		/*!<	Reset value for LOFF_FLIP register.	*/

/**
 *  \brief LOFF_SENSN LOFFNn bit mask definitions
 *	Default value, 0, _NOT_FLIPPED.
 */
#define		ADS1299_LOFF_FLIP8_FLIPPED		(1<<7)			/*!<	IN8P is pulled to AVSS and IN8N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP7_FLIPPED		(1<<6)			/*!<	IN7P is pulled to AVSS and IN7N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP6_FLIPPED		(1<<5)			/*!<	IN6P is pulled to AVSS and IN6N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP5_FLIPPED		(1<<4)			/*!<	IN5P is pulled to AVSS and IN5N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP4_FLIPPED		(1<<3)			/*!<	IN4P is pulled to AVSS and IN4N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP3_FLIPPED		(1<<2)			/*!<	IN3P is pulled to AVSS and IN3N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP2_FLIPPED		(1<<1)			/*!<	IN2P is pulled to AVSS and IN2N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP1_FLIPPED		(1<<0)			/*!<	IN1P is pulled to AVSS and IN1N pulled to AVDD	*/
#define		ADS1299_LOFF_FLIP8_NOT_FLIPPED		0		/*!<	IN8P is pulled to AVDD and IN8N pulled to AVSS */
#define		ADS1299_LOFF_FLIP7_NOT_FLIPPED		0		/*!<	IN7P is pulled to AVDD and IN7N pulled to AVSS */
#define		ADS1299_LOFF_FLIP6_NOT_FLIPPED		0		/*!<	IN6P is pulled to AVDD and IN6N pulled to AVSS */
#define		ADS1299_LOFF_FLIP5_NOT_FLIPPED		0		/*!<	IN5P is pulled to AVDD and IN5N pulled to AVSS */
#define		ADS1299_LOFF_FLIP4_NOT_FLIPPED		0		/*!<	IN4P is pulled to AVDD and IN4N pulled to AVSS */
#define		ADS1299_LOFF_FLIP3_NOT_FLIPPED		0		/*!<	IN3P is pulled to AVDD and IN3N pulled to AVSS */
#define		ADS1299_LOFF_FLIP2_NOT_FLIPPED		0		/*!<	IN2P is pulled to AVDD and IN2N pulled to AVSS */
#define		ADS1299_LOFF_FLIP1_NOT_FLIPPED		0		/*!<	IN1P is pulled to AVDD and IN1N pulled to AVSS */



/********************************************************
 *						  GPIO		    				*
 *      Configures actions of the three GPIO pins.	    *
 ********************************************************/
// Ska inte användas i projektet va?
//#define ADS1299_GPIO_DATA


/********************************************************
 *					   @CONFIG4		    				*
 *      Configures conversion mode and enables		    *
 * 			     lead-off comparators		    		*
 ********************************************************/
 #define ADS1299_CONFIG4_RESET		0x00;		/*!<	Reset value for CONFIG4 register.	*/

/**
 *  \brief CONFIG4 SINGLE_SHOT bit mask definitions
 *	Sets the conversion mode
 */
#define 		ADS1299_CONFIG4_SINGLE_SHOT_DISABLE		(0<<3)		/*!<	Continuous conversion mode	*/
#define 		ADS1299_CONFIG4_SINGLE_SHOT_ENABLE		(1<<3)		/*!<	Single-shot mode	*/

/**
 *  \brief CONFIG4 PD_LOFF_COMP bit mask definitions
 *	Powers down the lead-off comparators
 */
#define 		ADS1299_CONFIG4_PD_LOFF_COMP_DISABLE	(0<<1)		/*!<	Lead-off comparators disabled	*/
#define 		ADS1299_CONFIG4_PD_LOFF_COMP_ENABLE		(1<<1)		/*!<	Lead-off comparators disabled	*/





/*******************************************************************************************************************************************************************/
/************************************************					FUNCTIONS					********************************************************************/
/*******************************************************************************************************************************************************************/

/*
 * 	@ADS_FUNC
 * 	ADS1299 related functions
 */
void ADS_PowerOn();										/*!<	Powers on with normal electrod on channel 1.	*/
void ADSPowerOnTest();									/*!<	Powers on with test signal on CH1, this is used for testing the hardware		*/
void ADS_Transmit(uint8_t* data, uint16_t size);		/*!<	Transmits data to ADS	*/
void ADS_DRDY_Wait();									/*!<	Waits for DRDY to go active	*/
void ADS_ReadReg(uint8_t baseAddr, uint8_t numOfReg);	/*!<	Reads value of register(s)  */
void ADS_RReg(uint8_t baseAddr);						/*!<	Reads value of single register */
void ADS_WriteReg(uint8_t address, uint8_t val);		/*!<	Writes value to single register */


//	Commands
void ADS_WAKEUP();
void ADS_STANDBY();
void ADS_RESET();
void ADS_START();
void ADS_STOP();
void ADS_RDATAC();
void ADS_SDATAC();

// Writing to registers
void ADS_CHANNEL(uint8_t CHN_ADDR, uint8_t SETTINGS);
void ADS_CONFIG1(uint8_t bitmask);
void ADS_CONFIG2(uint8_t bitmask);
void ADS_CONFIG3(uint8_t bitmask);
void ADS_CONFIG4(uint8_t bitmask);
void ADS_BIAS_SENS(uint8_t SENSN, uint8_t SENSP);
// Actions
void ADS_DOUT();

void ADS_Send();
void ADS_Plot();
void ADS_bufferplot();

//TODO: below for testing, remove when done
void ADS_test();

void hodl();



// TODO: Are these needed for DOUT_t?
//#define 	ADS_NUM_OF_CHANNELS 8		/*!<	Total amount of channels for ADS1299	*/
//#define		ADS_CH_1			0		/*!<		*/

/*
 *	\brief Struct for received data from DOUT on the ADS1299
 *	Read more about DOUT in ADS1299 datasheet (rev. SBAS499C) p.39
 *	TODO: Maybe change to channel[8][3]; for easier handling?
 */
typedef struct
{
	uint8_t Status[3];					/*!<	24 bit status message	*/
	uint8_t Channel_1[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_2[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_3[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_4[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_5[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_6[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_7[3];		/*!<	24 bit data holder for data sent from slave to master		*/
	uint8_t Channel_8[3];		/*!<	24 bit data holder for data sent from slave to master		*/
}DOUT_t;


/*
 * 		User defined variable. Change depend on your setup
 */
extern SPI_HandleTypeDef hspi3;					/*!<	SPI channel for ADS1299	*/
extern DMA_HandleTypeDef hdma_spi3_rx;				/*!<	SPI DMA for ADS1299	*/
extern UART_HandleTypeDef huart2;					/*!<	For communicating over UART, not explicitly needed	*/

/*
 * 	Global variables
 */
extern DOUT_t DOUT;		/*!<	For storing data ouput from ADS		*/

// Todo: Fix. Add struct when errythang is gut
extern char ADSBuffer[1024];
extern uint8_t PlotBuffer[1024][3];
extern int plotCounter;
void ADS_DRDY_GPIO_EXTI(uint16_t ADS_N_DRDY_PIN);

//extern uint32_t cCounter;

#endif /* INC_ADS1299_DRIVERS_H_ */
