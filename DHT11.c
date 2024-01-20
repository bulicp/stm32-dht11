/*
 * DHT.c
 *
 *  Jan 17, 2024
 *  Pa3cio, UL FRI
 */


#include "stm32h7xx_hal.h"
#include "DHT11.h"
#include "DWT.h"


// change/define as necessary
#define DHT_PORT GPIOE
#define DHT_PIN GPIO_PIN_3


static inline void Set_DHT_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


static inline void Set_DHT_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


/*
 * Start and check for response from DHT11
 * 19 Jan 2024, Pa3cio, UL FRI
 * This function uses the DWT CYCCNT and expects the CYCCNT is running.
 * Otherwise fails.
 *
 * ret: 0 - DHT not present
 *      1 - DHT present
 */
static unsigned char DHT11_Start_and_Check_Response (){
	unsigned char bDHT_present = 0;

	// initialize DWT cycle counter
	//   you can do this only once (e.g. in main code) and remove it from here
	if (DWT_Init() != 0) return 0;	// error: clock cycle counter not started

	// Start bit - the host pulls the data line low for at leas 18 ms:
	Set_DHT_Pin_Output (DHT_PORT, DHT_PIN);  	// set the pin as output
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 0);   // pull the pin low
	DWT_DelayUS (20000);   // wait > 18ms

	// check if DHT11 present
	// the host pulls the data line high and waits for the responce from DHT11
	HAL_GPIO_WritePin (DHT_PORT, DHT_PIN, 1);   // pull the pin high
	DWT_DelayUS (10);   						// pull line high  and
	Set_DHT_Pin_Input(DHT_PORT, DHT_PIN);    	// set the line as input

	// Once DHT11 detects the Start Signal, it will send out a low voltage level response
	// signal, which lasts 80us. Then it sets the voltage level from low to high and keeps it for 80us.
	// Wait for 40 us
	DWT_DelayUS (40);
	// Read the line, it must be low at this point
	if (!(HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))
	{
		// if the data line is LOW, wait for 80 us:
		DWT_DelayUS (80);
		// Read the line, it must be HIGH at this point
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN))) bDHT_present = 1;
		else bDHT_present = 0;
	}
	// Now data transmission will start.
	return bDHT_present;
}


/*
 * Read 8 bits from DHT11
 * 19 Jan 2024, Pa3cio, UL FRI
 *
 * This function expects the DHT11 sensor to change the level of data signal according to specifications
 * If more than 100 us elapses without the level changing then the call fails with -1.
 * The function also uses the DWT CYCCNT and expects the CYCCNT is running. Otherwise fails with -2
 *
 *
 * ret:  0 - SUCCESS
 *      -1 - TIMEOUT
 *      -2 - DWT CYCCNT not running
 */
static char DHT11_Read_8bit (unsigned char *read_data) {
	unsigned char data = 0;

	unsigned int cpu_freq = HAL_RCC_GetSysClockFreq();
	unsigned int cycles_per_us = (cpu_freq / 1000000);
	unsigned int timeout = 100*cycles_per_us;			// timeout = 100 us
	unsigned int cyccnt = 0;

	// check if CYCCNT is running:
	DWT->CYCCNT = 0;
	cyccnt=DWT->CYCCNT;	// read CYCCNT
	__NOP();	// wait for 3 NOPs
	__NOP();
	__NOP();
	// check if CYCCNT has changed:
	if (!(DWT->CYCCNT > cyccnt)) return -2; // error: CYCCNT isn't running:


	// after the response, DHT will start the transmission.
	// Each bit’s transmission begins with low-voltage-level that last 50 us,
	// the following high-voltage-level signal’s length decides whether the bit is “1” or “0”.

	// Every bit of data begins with the 50us low voltage level and then switch
	// to high voltage level; high voltage level duration depends on the bit
	// value that have to be transmitted: a 1 bit has an high voltage level
	// duration of 27uS, a 0 bit has an high voltage level duration of 70uS.
	for (unsigned char j=0;j<8;j++)
	{

		DWT->CYCCNT = 0;
		while ( HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) ){	// wait for the pin to go low
			if (DWT->CYCCNT > timeout) return -1; 		// timeout error
		}

		DWT->CYCCNT = 0;
		while ( !(HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN)) ){   // wait for the pin to go high
			if (DWT->CYCCNT > timeout) return -1; 			// timeout error
		}
		// If the length of H is around 26-28 us, the bit is “0”
		// If the length of H is around 70 us, the bit is “1”

		// wait for 40 us and check the line:
		DWT_DelayUS (40);
		data = data << 1;
		if ((HAL_GPIO_ReadPin (DHT_PORT, DHT_PIN)))   // if the pin is HIGH
		{
			data |= 0x01;
		}
	}

	// When the last bit data is transmitted, DHT11 pulls down the voltage level and
	// keeps it for 50us, then it leaves the line pulled-up and goes back in
	// the stand-by mode.
	// To make another sensor read it needs to repeat this cycle, sending again the
	// Start Signal after a minimum of one second from the previous cycle.

	//return read data
	*read_data = data;
	return 0; // SUCCESS
}



/*
 * Read data from DHT11
 * 19 Jan 2024, Pa3cio, UL FRI
 * Be careful if using in a RTOS task! The function returns error if interrupted.
 * Lock the kernel before calling this function and unlock the kernel after
 * e.g. : osKernelLock (void), osKernelUnlock (void)
 *
 * ret:  0 - SUCCESS
 *      -1 - ERROR
 */
char DHT11_ReadData (DHT_DataTypedef *DHT_Data)
{
	unsigned char Rh_int = 0;
	unsigned char Rh_dec = 0;
	unsigned char Temp_int = 0;
	unsigned char Temp_dec = 0;
	unsigned char ChkSum = 0;

	unsigned char err = 0;


	if(DHT11_Start_and_Check_Response()){

		// Read 5 bytes of data according to the specifications:

		err |= DHT11_Read_8bit(&Rh_int);		// read Rh integer part
		err |= DHT11_Read_8bit(&Rh_dec);		// read Rh decimal part
		err |= DHT11_Read_8bit(&Temp_int);		// read T integer part
		err |= DHT11_Read_8bit(&Temp_dec);		// read T decimal part
		err |= DHT11_Read_8bit(&ChkSum);		// read checksum

		if (err) return -1; // error reading


		// checksum shoud be equal to the least significant 8 bits of Rh_int+Rh_dec+T_int+T_dec
		if (ChkSum == (unsigned char)(Rh_int + Rh_dec + Temp_int + Temp_dec)){

			DHT_Data->Temperature = Temp_int;

			if (Temp_dec & 0x80) {
				DHT_Data->Temperature = -1.0 - DHT_Data->Temperature;
			}
			DHT_Data->Temperature += (float)(Temp_dec & 0x7f) * 0.1;

			DHT_Data->Humidity = (float)Rh_int + Rh_dec * 0.1;

			return 0;
		}
		else return -1; // error checksum
	}
}


