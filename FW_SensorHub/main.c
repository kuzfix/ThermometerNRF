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
Wake - wake up the receiver - no action if already awake
Fxx! - Forward with speed xx: ASCII 0(stop)-99(max speed)
Bxx! - Backwards with speed xx (same format as F)
Txx! - Time to auto stop (xx seconds).
txx! - Time to auto stop if command receifed frome remote controller
sxxx - Time to auto stop if command slow receifed frome remote controller(xx.x seconds)
*/
void DecodeCommand(uint8_t *pu8_cmd)
{
  switch (pu8_cmd[0])
  {
    case 'T': 
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
