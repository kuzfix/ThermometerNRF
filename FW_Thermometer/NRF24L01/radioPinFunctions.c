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

#include <avr/io.h>

#define USE_HARDWARE_SPI

//IO pin register and bit mapping
#define NRF24_DDR_MISO	DDRD
#define NRF24_PORT_MISO	PORTD
#define NRF24_PIN_MISO	PIND
#define NRF24_BIT_MISO	0
#define NRF24_DDR_MOSI	DDRD
#define NRF24_PORT_MOSI	PORTD
#define NRF24_BIT_MOSI	1
#define NRF24_DDR_SCK	  DDRD
#define NRF24_PORT_SCK	PORTD
#define NRF24_BIT_SCK	  2
#define NRF24_DDR_CS	  DDRD
#define NRF24_PORT_CS	  PORTD
#define NRF24_BIT_CS	  5
#define NRF24_DDR_CE	  DDRD
#define NRF24_PORT_CE	  PORTD
#define NRF24_BIT_CE	  4

//IRQ pin (currently not used)
#define NRF24_DDR_IRQ	  DDRD
#define NRF24_PORT_IRQ	PORTD
#define NRF24_BIT_IRQ	  3

//SPI register and bit remapping (some have SPCR, others SPCR0, etc.)
/**/


#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
  set_bit(NRF24_DDR_CE,NRF24_BIT_CE); // CE output
  set_bit(NRF24_DDR_CS,NRF24_BIT_CS); // CSN output
  set_bit(NRF24_DDR_SCK,NRF24_BIT_SCK); // SCK output
  set_bit(NRF24_DDR_MOSI,NRF24_BIT_MOSI); // MOSI output
  clr_bit(NRF24_DDR_MISO,NRF24_BIT_MISO); // MISO input

#ifdef USE_HARDWARE_SPI
    UCSRA = 0;
    UCSRB = (1<<RXEN) | (1<<TXEN);
#if defined(UMSEL)
    UCSRC = (1<<UMSEL) | (0<<UPM0) | (3<<UCSZ0) | (0<<UCPOL);
#else
    UCSRC = (1<<UMSEL0) | (0<<UPM0) | (3<<UCSZ0) | (0<<UCPOL);
#endif
#endif
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(NRF24_PORT_CE,NRF24_BIT_CE);
    }
    else
    {
        clr_bit(NRF24_PORT_CE,NRF24_BIT_CE);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(NRF24_PORT_CS,NRF24_BIT_CS);
    }
    else
    {
        clr_bit(NRF24_PORT_CS,NRF24_BIT_CS);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(NRF24_PORT_SCK,NRF24_BIT_SCK);
    }
    else
    {
        clr_bit(NRF24_PORT_SCK,NRF24_BIT_SCK);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(NRF24_PORT_MOSI,NRF24_BIT_MOSI);
    }
    else
    {
        clr_bit(NRF24_PORT_MOSI,NRF24_BIT_MOSI);
    }
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
    return check_bit(NRF24_PIN_MISO,NRF24_BIT_MISO);
}
/* ------------------------------------------------------------------------- */

#ifndef USE_HARDWARE_SPI
extern uint8_t spi_transfer_SW(uint8_t tx);
#endif
uint8_t spi_transfer(uint8_t tx)
{
#ifdef USE_HARDWARE_SPI
  uint8_t rx = 0;

  UDR = tx;
  while ( !(UCSRA & (1<<RXC)) ) {} //wait until done
  rx = UDR;
  return rx;
#else
  return spi_transfer_SW(tx);
#endif
}
/* ------------------------------------------------------------------------- */
