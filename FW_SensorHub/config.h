/*
 * config.h
 *
 * Created: 3.6.2015 14:07:32
 *  Author: maticpi
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 16000000UL
#define BAUD1 9600UL
#define BAUD0 57600UL	//115200 has too high error with 16MHz crystal

//RX and TX addresses, and RF channel
// One set of addresses is used for all devices, which are communicating with
// one sensor hub (so the hub does not need to know which sensor it is speaking
// to. In this application that information is stored in the ID field of the
// payload.
//Sensors connected to the indoor hub "Pec":
//#define NRF24_RX_ADDRESS	{0xD7,0xD7,0xD7,0xD7,0xD7}
//#define NRF24_TX_ADDRESS	{0xE7,0xE7,0xE7,0xE7,0xE7}
//#define NRF24_RF_CHANNEL	2
//Sensor connected to the 1st outdoor hub "Vrt":
#define NRF24_RX_ADDRESS	{0xA5,0xA5,0xA5,0xA5,0xA5}
#define NRF24_TX_ADDRESS	{0xC3,0xC3,0xC3,0xC3,0xC3}
#define NRF24_RF_CHANNEL	2

#endif /* CONFIG_H_ */