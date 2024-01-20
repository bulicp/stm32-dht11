/*
 * DHT.h
 *
 *  Jan 17, 2024
 *  Pa3cio, UL FRI
 */

#ifndef DHT11_H_
#define DHT11_H_

typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;


//unsigned char DHT11_Start_and_Check_Response ();
//unsigned char DHT11_Read_8bit ();

/*
 * This is the only exported function of the driver.
 * It incorporates the initialization, start, presence detection and
 * data read.
 * Pa3cio, UL FRI, 20 Jan 2024
*/
char DHT11_ReadData (DHT_DataTypedef *DHT_Data);

#endif /* INC_DHT11_H_ */
