/* ----------------------------------------------------
 * File    : uart.c
 * Author  : Alex Weber, alex@tinkerlog.com, http://tinkerlog.com
 * Hardware: ATmega8, 4.096MHz
 * Software: WinAVR20070525
 * 
 * Adapted from:
 * Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
 */

#include <inttypes.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "uart.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0 
#endif



#define BAUD 19200
//#define BAUD 4800
// 4.096MHz
//  4800: 52.3333333
//  9600: 25.6666667
// 14400: 16.7777778
// 19600: 12.06
// 28800: 7.8889
// 38400: 5.6667

#define MYUBBR ((F_CPU / (BAUD * 16L)) - 1)
#define TX_BUFFER_SIZE 32
#define RX_BUFFER_SIZE 32
#define RX_BUFFER_MASK ( RX_BUFFER_SIZE - 1 )
#define TX_BUFFER_MASK ( TX_BUFFER_SIZE - 1 )
#if ( RX_BUFFER_SIZE & RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif
#if ( TX_BUFFER_SIZE & TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif


static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;


/*
 * init_uart
 */
void init_uart(void) {
	// set baud rate
	UBRRH = (uint8_t)(MYUBBR >> 8); 
	UBRRL = (uint8_t)(MYUBBR);
	
	// enable receive and transmit
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	// set frame format
	UCSRC = (1 << URSEL) | (3 << UCSZ0);	// asynchron 8n1
}



/*
 * send_uart
 * Sends a single char to UART
 */
//void send_uart(uint8_t c) {
void _uart_putc(uint8_t c) {
	// wait for empty data register
	while (!(UCSRA & (1 << UDRE)));	
	// set data into data register
	UDR = c;
}



/*
 * uart_getc_f
 * getc in stdio style.
 */
int uart_getc_f(FILE *stream) {
	uint16_t c;
	while ((c = uart_getc()) == UART_NO_DATA) {}
	return c;
}

uint8_t uart_getc_wait(void) {
	uint16_t c;
	while ((c = uart_getc()) == UART_NO_DATA) {}
	return c;
}



/*
 * uart_getc
 * Gets a single char.
 * return	uint16_r	the received char or UART_NO_DATA 
 */
uint16_t uart_getc(void) {
	uint8_t c = 0;
	uint8_t tmp_tail = 0;
	if (rx_head == rx_tail) {
		return UART_NO_DATA;
	}
	tmp_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
	c = rx_buffer[rx_tail];
	rx_tail = tmp_tail;
	return c;
}



/*
 * SIGNAL RX complete
 * Receives a char from UART and stores it in ring buffer.
 */
SIGNAL(USART_RXC_vect) {
	uint8_t tmp_head = 0;
	uint8_t data = UDR;
	tmp_head = (rx_head + 1) % RX_BUFFER_SIZE;
	if (tmp_head == rx_tail) {
		// buffer overflow error!
	}
	else {
		rx_buffer[rx_head] = data;
		rx_head = tmp_head;
	}
}



/*
 * uart_putc_f
 * Puts a single char. Used by printf functions.
 */
int uart_putc_f(char c, FILE *stream) {
	uart_putc(c);
    return 0;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s);
        s++;
    }
}
void uart_puts_P(const char *s) {
	while (pgm_read_byte(s) != 0x00) {
		uart_putc(pgm_read_byte(s++));
	}
}



/*
 * Puts a single char.
 *   uint8_t c	the char to transmit
 */
void uart_putc(uint8_t c) {
	uint8_t tmp_head = 0;
	tmp_head = (tx_head + 1) & TX_BUFFER_MASK;
	// wait for space in buffer
	while (tmp_head == tx_tail) {
		;
	}
	tx_buffer[tx_head] = c;
	tx_head = tmp_head;
	// enable uart data interrupt (send data)
	UCSRB |= (1<<UDRIE);
}



/*
 * SIGNAL User Data Regiser Empty
 * Send a char out of buffer via UART. If sending is complete, the 
 * interrupt gets disabled.
 */
SIGNAL(USART_UDRE_vect) {
	uint8_t tmp_tail = 0;
	if (tx_head != tx_tail) {
		tmp_tail = (tx_tail + 1) & TX_BUFFER_MASK;
		UDR = tx_buffer[tx_tail];
		tx_tail = tmp_tail;
	}
	else {
		// disable this interrupt if nothing more to send
		UCSRB &= ~(1 << UDRIE);
	}
}



