/*
 * FW_VlakecDuplo.c
 *
 * Created: 09-Dec-18 14:02:02
 * Author : maticpi
 */ 

#include <avr/io.h>
#include <string.h>
#include "config.h"
#include "ADC.h"
#include "systime.h"
#include "UART0_IRQ.h"  //for debug
#include "nrf24.h"
#include "kbd.h"

#include "XPT2046.h"
#include "LCD_Ili9341.h"
#include "onewire.h"
#include "ds18x20.h"
#include "crc8.h"
/#include "MAX6675.h"

#define DEFAULT_AWAKE_TIME (10*60*1000UL)  //10 min

void InitIO()
{
  PORTE=0xFF;
  PORTC=0xF9; //PC1 = ADC power sense, PC2 = BTN (pulldown, short to VCC)
  PORTD=0x4F; //PD7 = CE(NRF24..), PD2=IRQ, PD1=TX,PD0=RX, PD5=DbgLED, PD6=start/stop sound
  PORTB=0x11; //PB0 = nCS, PB1,2=motA,B, PB3=MOSI, PB4=MISO, PB5=SCK (PB6,7=XTAL)
  //1-out, 0-in
  DDRE=0x00;
  DDRC=0x00;
  DDRD=0xa2;
  DDRB=0x2F;
}

uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

/* PROTOCOL
ID valH valL Vdd rtr
ID: 7..3 = ID, 2..0 = sensor type
valH - raw value HIGH byte.
valL - raw value LOW byte.
Vdd - power supply voltage
rtr - number of retransmissions of the previous packet (indicates ink quality, lower is better)
*/
#define STYPE_Tds18b20		0x01
#define STYPE_Tsht21			0x02
#define STYPE_Hsht21			0x03
#define STYPE_Tbmp280			0x05
#define STYPE_Pbmp280			0x06

void DecodeCommand(uint8_t *pu8_cmd)
{
  uint8_t ID = pu8_cmd[0]>>3;
  float sensor_val; 
  switch (pu8_cmd[0] & 0x07)
  {
    case STYPE_Tds18b20: 
      printf(" T=%d.%02dC (%3d)", pu8_cmd[1], (pu8_cmd[2]*100)/256, pu8_cmd[3] ); 
      break;
    default: 
      printf(" ***%c_%02X.%02X-%d", pu8_cmd[0], pu8_cmd[1], pu8_cmd[2], pu8_cmd[3]); 
      break;
  }

}

int main(void)
{
  uint8_t   pu8_data[33];
  
  InitIO();
  KBD_Init();
  Systime_Init();
  ADC_Init();
  DIDR0=(1<<1); //ADC_Init disables input buffers for PC0 - PC3. Only disable the actual analog inputs dig buf
  UART0_Init();
  stdout = &UART0_str;
  sei();

  nrf24_init();                   // init hardware pins 
  nrf24_config(2,4);              // Channel #2 , payload length: 4 
  nrf24_tx_address(tx_address);   // Set the device addresses 
  nrf24_rx_address(rx_address);

  printf("Start:");
  while (1)
  {
    if (HasOneMillisecondPassed())
    {
      KBD_Read();
      
      if(nrf24_dataReady())
      {
        printf(" D:");
        memset(pu8_data,0x00,33);
        nrf24_getData(pu8_data);
        DecodeCommand(pu8_data);
  	  }
    }

  }
  
}
