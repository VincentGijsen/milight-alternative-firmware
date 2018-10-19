#include "delay.h"




void delay_ms(unsigned short ms){
  unsigned short d;
  for(d = 0 ; d<ms; d++){
    delay_us(1000);
  }
}

void delay_cycles(unsigned short cycles)
{
  unsigned short d;
  for (d = 0; d < cycles; ++d)
  {
    nop();
  }
}

