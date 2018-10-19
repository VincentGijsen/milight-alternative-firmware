//main.h
#ifndef __VINCENT_MAIN_H
#define __VINCENT_MAIN_H


//#define BREAD
#define RGBWMODULE




#ifdef BREAD
#define INTERNALTIMER
#warning USING Breadboard Wireing

#define LT8900_PORT_CLK         GPIOD //pin1
#define PIN_BB_CLK              GPIO_PIN_4 //pin1

#define LT8900_PORT_MOSI        GPIOD//pin2
#define PIN_BB_MOSI             GPIO_PIN_5 //pin2

#define LT8900_PORT_MISO        GPIOD //pin20
#define PIN_BB_MISO             GPIO_PIN_3 //pin20

#define LT8900_PORT_SS          GPIOD //pin1
#define LT8900_PIN_SS           GPIO_PIN_6 //pin1

#define LT8900_PORT_PKT         GPIOD //pin 19
#define LT8900_PIN_PKT          GPIO_PIN_2 //pin15

#define LT8900_PORT_RESET       GPIOC //to vcc connected
#define LT8900_PIN_RESET        GPIO_PIN_5 //?

#define LT8900_PORT_DVSS        GPIOD //pin3
#define LT8900_PIN_DVSS         0 //


#endif


#ifdef RGBWMODULE
#define CRYSTALTIMER
#warning USING MiLight RGBW ledstrip Driver
/*
pinout 
from pl1167 based on authometion
1: AVSS
2: NC
3: pkt
4: reset
5: ground
6: SS(SCSB)
7: SCK
8: mosi
9: miso
10:MODE
11:VCC
12:VDDO
13:XOUT
14:XIN
15:ANTB
16:ANTA

THIS IS DIFFERENT FROM THE DATASHEET!!!!!

*/

#define LT8900_PORT_CLK         GPIOD //pin1
#define PIN_BB_CLK              GPIO_PIN_4 //pin1

#define LT8900_PORT_MOSI        GPIOD//pin2
#define PIN_BB_MOSI             GPIO_PIN_5 //pin2

#define LT8900_PORT_MISO        GPIOD //pin20
#define PIN_BB_MISO             GPIO_PIN_3 //pin20

#define LT8900_PORT_SS          GPIOD //pin1
#define LT8900_PIN_SS           GPIO_PIN_6 //pin1

#define LT8900_PORT_PKT         GPIOD //pin 19
#define LT8900_PIN_PKT          GPIO_PIN_2 //pin15

#define LT8900_PORT_RESET       GPIOC //to vcc connected
#define LT8900_PIN_RESET        GPIO_PIN_5 //?

#define LT8900_PORT_DVSS        GPIOD //pin3
#define LT8900_PIN_DVSS         0 //


#endif


  
#endif /*__VINCENT_MAIN_H*/