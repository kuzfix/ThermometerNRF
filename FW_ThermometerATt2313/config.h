/*
 * config.h
 *
 * Created: 3.6.2015 14:07:32
 *  Author: maticpi
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

#define F_CPU 1000000UL
#define BAUD1 9600UL
#define BAUD0 9600UL

#define USE_DS18B20
#define MAXSENSORS 1
#define OW_ONE_BUS
//DS18B20 pin set in onwire.h - PB1

//#define USE_SHT2x


#endif /* CONFIG_H_ */