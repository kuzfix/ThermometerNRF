/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include "main.h"

#define USE_HARDWARE_SPI
extern SPI_HandleTypeDef hspi1;

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
/*  set_bit(NRF24_DDR_CE,NRF24_BIT_CE); // CE output
  set_bit(NRF24_DDR_CS,NRF24_BIT_CS); // CSN output
  set_bit(NRF24_DDR_SCK,NRF24_BIT_SCK); // SCK output
  set_bit(NRF24_DDR_MOSI,NRF24_BIT_MOSI); // MOSI output
  clr_bit(NRF24_DDR_MISO,NRF24_BIT_MISO); // MISO input

#ifdef USE_HARDWARE_SPI
	NRF24_SPCR = (0<<NRF24_SPIE) | (1<<NRF24_SPE) | (0<<NRF24_DORD) | (1<<NRF24_MSTR) | (0<< NRF24_CPOL) | (0<<NRF24_CPHA) | (0<<NRF24_SPR0);
	NRF24_SPSR = (1<<NRF24_SPI2X);
#endif*/
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
	if(state)
	{
		HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(CE_GPIO_Port,CE_Pin,GPIO_PIN_RESET);
	}
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
	if(state)
	{
		HAL_GPIO_WritePin(nCS_GPIO_Port,nCS_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(nCS_GPIO_Port,nCS_Pin,GPIO_PIN_RESET);
	}
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{/*
	if(state)
	{
		HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_RESET);
	}*/
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{/*
	if(state)
	{
		HAL_GPIO_WritePin(MOSI_GPIO_Port,MOSI_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(MOSI_GPIO_Port,MOSI_Pin,GPIO_PIN_RESET);
	}*/
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
	return 0; //HAL_GPIO_ReadPin(MISO_GPIO_Port, MISO_Pin);
}
/* ------------------------------------------------------------------------- */

#ifndef USE_HARDWARE_SPI
extern uint8_t spi_transfer_SW(uint8_t tx);
#endif
uint8_t spi_transfer(uint8_t tx)
{
#ifdef USE_HARDWARE_SPI
  uint8_t rx;
	
	HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10);

  return rx;
#else
  return spi_transfer_SW(tx);
#endif
}
/* ------------------------------------------------------------------------- */
