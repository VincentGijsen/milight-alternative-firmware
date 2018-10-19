#include "stm8s.h"
#include <string.h>
#include <stdio.h>
#include "stm8s_gpio.h"
#include "stm8s_uart1.h"

/*
* PD5: UART1_TX
* PD6: UART1_RX
 */
void Uart_Init(){
  
  UART1_DeInit();
  UART1_Init((u32)115200, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO, UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TXRX_ENABLE);
}

void SerialPutChar( char c){
    UART1_SendData8(c);
    while ((UART1->SR & UART2_SR_TXE ) != UART1_SR_TXE );
}

void SerialPutString( char *s){
    while (*s != '\0'){
        SerialPutChar(*s);
        s++;
    }
}