/*
Author: Vincent Gijsen
*/

#ifndef SPI_BITBANG_H
#define SPI_BITBANG_H

/*
#define MODE0 0
#define MODE1 1
#define MODE2 2
#define MODE3 3

#define SLOW 0
#define FAST 1
*/

class SPI_BitBang{
public:
  enum DataRate{
    SLOW,
    FAST
  };
  enum DataMode{
    MODE0,
    MODE1,
  };
  void begin(void);
  SPI_BitBang(GPIO_TypeDef * pClk, GPIO_TypeDef * pMosi, GPIO_TypeDef * pMiso, const uint8_t clk, const uint8_t mosi, const uint8_t miso); 
  //uint8_t transfer(unsigned char  data);
  void setDataMode(DataMode mode);
  void setDataRate(DataRate rate);
  uint8_t read(uint8_t data);
  void write(uint8_t data);
  uint8_t transfer(uint8_t data);
  
private:
  
  GPIO_TypeDef *_pClk;
  GPIO_TypeDef *_pMiso;
  GPIO_TypeDef *_pMosi;
  uint8_t _clk; 
  uint8_t _mosi ;
  uint8_t _miso ;
  
  uint8_t _delay ;
  
  DataRate _rate;
  DataMode _mode;
};

#endif /* SPI_BITBANG_H*/