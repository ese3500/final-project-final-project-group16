#define F_CPU 16000000UL
#ifndef UART_H
#define UART_H
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void UART_init();

void UART_send( unsigned char data);

void UART_putstring(char* StringPtr);

#endif

void UART_init()
{
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	//Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 2 stop bits, 8 data bits */
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8 data bits
	UCSR0C |= (1<<USBS0); // 2 stop bits
}

void UART_send(unsigned char data)
{
	// Wait for empty transmit buffer
	while(!(UCSR0A & (1<<UDRE0)));
	// Put data into buffer and send data
	UDR0 = data;
	
}

void UART_putstring(char* StringPtr)
{
	while(*StringPtr != 0x00)
	{
		UART_send(*StringPtr);
		StringPtr++;
	}
}
char String[25];

void Initialize()
{
	cli();
	
	//GPIO
	DDRB |= (1 << DDB1);
	
	//Clear on Compare Match
	TCCR1A |= (1<<COM1A1);
	TCCR1A &= ~(1<<COM1A0);
	
	//256 prescaling
	TCCR1B |= (1<<CS12);
	TCCR1B &= ~(1<<CS11);
	TCCR1B &= ~(1<<CS10);
	
	//Mode 14 fast pwm
	TCCR1A &= ~(1<<WGM10);
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12);
	TCCR1B |= (1<<WGM13);
	
	//OCR1B = 63;
	ICR1 = 1249;
	OCR1A = 250;

	sei();
}

int main(void)
{	
	Initialize();
	while(1)
	{	
		
		OCR1A = 160; 
		_delay_ms(1000); 
		OCR1A = 30; 
		_delay_ms(1000); 
	}
}