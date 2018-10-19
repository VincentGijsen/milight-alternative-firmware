#ifndef LT8900_H
#define LT8900_H


#include "stm8s.h"
#include "stm8s_gpio.h"

#include "SPI_BitBang.h"
#include "delay.h"


class LT8900
{
  
public:
  /** Data Data
  * While the LT8900 only has support for 1MPBS speed, the LT8910 offers different speeds.
  * @see getIs8910
  */
  enum DataRate
  {
    LT8900_1MBPS,     /** default transmit rate */
    LT8910_250KBPS,   /** 250 Kpbs, only on lt8910 */
    LT8910_125KBPS,   /** 125 Kbps, only on lt8910 */
    LT8910_62KBPS     /** 62 Kbps, only on lt8910 */
  };
  
private:
  SPI_BitBang * _spi;
  GPIO_TypeDef * _port_ss;
  GPIO_TypeDef * _port_pkt;
  GPIO_TypeDef * _port_reset;
  GPIO_TypeDef * _port_dvss;
  uint8_t _pin_ss;
  uint8_t _pin_pkt;
  uint8_t _pin_reset;
  uint8_t _channel;
  bool _isLT8910;
  uint8_t _pin_dvss;
  
  
public:
  /** Construct a new instance
  * @param cs Chip Select, this is the SLAVE SELECT pin. Please note that it is a different location than the NRF24L01+ SS pin.
  * @param pkt PKT_flag input signal pin. Comparable to the NRF24L01+ IRQ pin.
  * @param rst RESET pin. Use 0 to disable.
  */
  LT8900(SPI_BitBang * spi, 
         GPIO_TypeDef * port_ss, 
         GPIO_TypeDef * port_pkt, 
         GPIO_TypeDef * port_reset, 
         GPIO_TypeDef * port_dvss,
         const uint8_t cs, 
         const uint8_t pkt, 
         const uint8_t rst,
         const uint8_t dvss);
  
  /** Configures the module for initial use */
  DataRate getDataRate();
  bool available(void);
  bool sendPacket(uint8_t *data, uint8_t packetSize);
  bool setDataRate(DataRate rate);
  int read(uint8_t *buffer, uint8_t maxBuffer);
  uint16_t readRegister(uint8_t reg);
  uint8_t writeRegister(uint8_t reg, uint16_t data);
  uint8_t writeRegister2(uint8_t reg, uint8_t high, uint8_t low);
  void begin();
  void setChannel(uint8_t channel);
  void setCurrentControl(uint8_t power, uint8_t gain);
  void setSyncWord(uint16_t a, uint16_t b, uint16_t c, uint16_t d);
  void startListening(void);
  void status();
};

#endif /* LT8900 */
