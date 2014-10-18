/* ----------------------------------------------------
 * File    : uart.h
 * Author  : Alex Weber, alex@tinkerlog.com, http://tinkerlog.com
 * Hardware: ATmega8, 4.096MHz
 * Software: WinAVR20070525
 * 
 * Adapted from:
 * Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
 */

#ifndef UART_H_
#define UART_H_

#include <stdio.h>

#define UART_NO_DATA 0x0100

/*
 * init_uart
 * Initialize UART to 19200 baud with 8N1. 
 */
void init_uart(void);

/* 
 * uart_getc
 * Sends a single char via UART.
 * return	uint16_r	the received char or UART_NO_DATA 
 */
uint16_t uart_getc(void);
uint8_t uart_getc_wait(void);
int uart_getc_f(FILE *stream);

/*
 * uart_putc
 * Puts a single char via UART.
 * uint8_t c	the char to transmit
 */
void uart_putc(uint8_t c);
int uart_putc_f(char c, FILE *stream);

/*
 * uart_puts
 * Sends a string.
 * The string is copied in a buffer. The buffer is transmitted using
 * interrupts.
 * This call blocks until the previous sending is complete.
 */
void uart_puts(const char *s);
void uart_puts_P(const char *s);

#endif /*UART_H_*/
