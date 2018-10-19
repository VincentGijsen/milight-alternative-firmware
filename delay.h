

#ifndef _UTIL_DELAY_H_
#define _UTIL_DELAY_H_ 
#include <limits.h>
#include "main.h" 

   
   

#ifdef INTERNALTIMER
#define F_CPU 16000000
#endif

#ifdef CRYSTALTIMER
#define F_CPU 12000000
#endif

#define delay_us(us) __delay_cycles((F_CPU  *(us)/10000000uL))


extern long volatile tickMs;

static inline void __delay_cycles(unsigned long us){
  while(us--){};
}


static inline void delay_ms( unsigned short __ms )
{
  long now = tickMs;
  long ends = tickMs + __ms;
  
  if( ends < now){
    //compensate for long overflow!
    while(tickMs <LONG_MAX) {
      //wait while timer has not overflowed
      asm("nop");
  }
  }
  //now just wait while tick < end period
    while(tickMs < ends){
      asm("nop");
    }
  
	
}

#endif