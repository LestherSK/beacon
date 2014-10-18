#ifndef SOFT_UART_H_
#define SOFT_UART_H_

#include <stdio.h>

#define SUART_TXD
#define SUART_RXD

extern void init_suart();

#ifdef SUART_TXD
    extern void suart_putc(const char);
	int suart_putc_f(char c, FILE *stream);
    
#endif // SUART_RXD

#ifdef SUART_RXD
    extern int suart_getc_wait();
    extern int suart_getc_nowait();
    int suart_getc_f(FILE *stream);
#endif // SUART_RXD

#endif /*SOFT_UART_H_*/
