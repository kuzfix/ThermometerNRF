/*
 * FW_Thermometer.c
 *
 * Created: 18/12/2019 19:47:43
 * Author : maticpi
 */ 

#include <avr/io.h>
#include "config.h"
#include <util/delay.h>
#include "nrf24.h"
#ifdef USE_DS18B20
  #include "onewire.h"
  #include "ds18x20.h"
#endif
#ifdef USE_SHT2x
  #define LM75ADDRESS 0x90
  #define I2C_READ  1
  #define I2C_WRITE 0
  #include "usi_i2c_master.h"
  #include "crc8.h"
#endif


int main(void)
{
#ifdef USE_DS18B20
  uint8_t u8_nSensors;
#endif
#ifdef USE_SHT2x
  char i2cmsg[5];
  uint8_t crc;
#endif
  int16_t i16Temp = (-99)<<8;
  int16_t i16Humidity = (-99)<<8;

  uint8_t q = 0;
  uint8_t temp;
  uint8_t data_array[4];
  uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
  uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};

#ifdef USE_DS18B20
  u8_nSensors = search_sensors();
#endif    
#ifdef USE_SHT2x
  USI_I2C_Init();
#endif
  /* init hardware pins */
  nrf24_init();
    
  /* Channel #2 , payload length: 4 */
  nrf24_config(2,4);

  /* Set the device addresses */
  nrf24_tx_address(tx_address);
  nrf24_rx_address(rx_address);

  /* Replace with your application code */
  while (1) 
  {
#ifdef USE_DS18B20
    if (u8_nSensors > 0)
    {
      DS18X20_start_meas(DS18X20_POWER_PARASITE,NULL);
      if ( DS18X20_read_meas_raw( &gSensorIDs[0][0], &i16Temp) != DS18X20_OK ) i16Temp=(-99)<<4;
      //if (i16Temp & 0x0800) i16Temp = (i16Temp | 0xf000); //expand leading 1s if negative
      i16Temp<<=4;  //high byte = integer part, low byte = fractional part
    }
#endif    
#ifdef USE_SHT2x
/*
    //LM75B - for i2c testing only
    i2cmsg[0]=LM75ADDRESS+I2C_WRITE;
    i2cmsg[1]=0;
    USI_I2C_Master_Start_Transmission(i2cmsg,2);
    i2cmsg[0]=LM75ADDRESS+I2C_READ;
    USI_I2C_Master_Start_Transmission(i2cmsg,3);
    i16Temp=((int)i2cmsg[1]<<8) | i2cmsg[2];
*/
    //SHT21
    #define SHT21_ADDRESS 0x80
    #define SHT21_MEASURE_T_HOLD 0xE3
    #define SHT21_MEASURE_RH_HOLD 0xE5
    
    i2cmsg[0]=SHT21_ADDRESS + I2C_WRITE;
    i2cmsg[1]=SHT21_MEASURE_T_HOLD;
    USI_I2C_Master_Start_Transmission(i2cmsg,2);
    i2cmsg[0]=SHT21_ADDRESS + I2C_READ;
    USI_I2C_Master_Start_Transmission(i2cmsg,4);
    i16Temp = (i2cmsg[1]<<8) | (i2cmsg[2] & ~0x03);
    crc = crc8(i2cmsg,2);
    if (i2cmsg[3] != crc) i16Temp = 0xDEAD;
    //use formula -46.85+175.72*i16Temp/65536 to get °C

    i2cmsg[0]=SHT21_ADDRESS + I2C_WRITE;
    i2cmsg[1]=SHT21_MEASURE_RH_HOLD;
    USI_I2C_Master_Start_Transmission(i2cmsg,2);
    i2cmsg[0]=SHT21_ADDRESS + I2C_READ;
    USI_I2C_Master_Start_Transmission(i2cmsg,4);
    i16Humidity = (i2cmsg[1]<<8) | (i2cmsg[2] & ~0x03);
    crc = crc8(i2cmsg,2);
    if (i2cmsg[3] != crc) i16Humidity = 0xDEAD;
    //use formula -6.5+125*i16Humidity/65536
#endif

    /* Fill the data buffer */
    data_array[0] = 0x00;
    data_array[1] = 0xAA;
    data_array[2] = 0x55;
    data_array[3] = q++;

    /* Automatically goes to TX mode */
    nrf24_send(data_array);
           
    /* Wait for transmission to end */
    while(nrf24_isSending());

    /* Make analysis on last tranmission attempt */
    temp = nrf24_lastMessageStatus();

    /* Retranmission count indicates the tranmission quality */
    temp = nrf24_retransmissionCount();

    /* Optionally, go back to RX mode ... */
    nrf24_powerUpRx();

    /* Or you might want to power down after TX */
    // nrf24_powerDown();

    /* Wait a little ... */
    _delay_ms(100);

  }
}

