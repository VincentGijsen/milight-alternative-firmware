#ifndef UART_H
#define UART_H

#include <string.h>
#include <stdio.h>

void SerialPutChar( char c);
void SerialPutString( char *s);

void Uart_Init(void);

#endif /* UART_H */
