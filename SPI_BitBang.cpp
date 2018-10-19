//--------------------------------------------------------------------------------
//
//  Bit bang data.
//
// Author: VIncent Gijsen
// STM8S103 / STM8S003
// BITBang SPI 
// 8 bit, MSB first, clock eval on traing rising edge
// INactive clock High, 
//



#include "main.h"
#include "stm8s_gpio.h"
#include "SPI_BitBang.h"
#include "delay.h"

#define DATA_LOW  GPIO_WriteLow(_pMosi, (GPIO_Pin_TypeDef)_mosi)
#define DATA_HIGH GPIO_WriteHigh(_pMosi, (GPIO_Pin_TypeDef)_mosi)

#define CLOCK_LOW GPIO_WriteLow(_pClk, (GPIO_Pin_TypeDef)_clk)
#define CLOCK_HIGH GPIO_WriteHigh(_pClk, (GPIO_Pin_TypeDef)_clk)

#define MISO GPIO_ReadInputPin(_pMiso, (GPIO_Pin_TypeDef)_miso)




SPI_BitBang::SPI_BitBang(GPIO_TypeDef * pClk, 
                         GPIO_TypeDef * pMosi, 
                         GPIO_TypeDef * pMiso,
                         const uint8_t clk, 
                         const uint8_t mosi, 
                         const uint8_t miso)
{
  _pClk = pClk;
  _pMosi = pMosi;
  _pMiso = pMiso;
  _clk = clk;
  _mosi = mosi;
  _miso = miso;
  
  _delay = 1;
  
  _mode = MODE0;
  _rate = FAST;
  
}

void SPI_BitBang::begin(){
  GPIO_Init(_pClk, (GPIO_Pin_TypeDef)_clk, GPIO_MODE_OUT_PP_LOW_SLOW);
  GPIO_Init(_pMosi, (GPIO_Pin_TypeDef)_mosi, GPIO_MODE_OUT_PP_LOW_SLOW); //output, pp
  GPIO_Init(_pMiso, (GPIO_Pin_TypeDef)_miso, GPIO_MODE_IN_FL_NO_IT); //init as input,floating, no interupt
  
  SPI_BitBang::setDataRate(_rate);
  
  CLOCK_LOW;
  DATA_LOW;
}


void SPI_BitBang::setDataMode(SPI_BitBang::DataMode mode){
  _mode = mode;
}

void SPI_BitBang::setDataRate(SPI_BitBang::DataRate rate){
  switch(rate){
  case SLOW:
    _delay = 100;
    break;
  case FAST: 
    _delay = 1;
    break;
  }
  
};




uint8_t SPI_BitBang::transfer(uint8_t b) 
{
  uint8_t res = 0;
  
  for(uint8_t _bit = 0;_bit < 8;_bit++)
  {
    
    CLOCK_HIGH;
    __delay_cycles(_delay);
    if (b&0x80)
      DATA_HIGH;
    else
      DATA_LOW;
    b <<= 1;
    
    
    
    CLOCK_LOW; 
    __delay_cycles(_delay);
    
    res <<=1;
    if(MISO !=0){
      res |=1;
    }
    __delay_cycles(_delay);
    
  }  
  return res;
}


