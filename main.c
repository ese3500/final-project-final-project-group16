#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define TRIG_PIN DDB5
#define ECHO_PIN DDB6

volatile uint16_t capture_count = 0;
volatile uint16_t start_time = 0;
volatile uint16_t overflow = 0;
volatile uint16_t end_time = 0;
volatile uint16_t distance_cm = 0;
volatile uint8_t is_continous = 1;
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((16000000UL / (BAUD_RATE * 16UL))) - 1)



void uart_init(void) {
	UBRR0H = (unsigned char)(BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	UCSR0B = (1 << TXEN0); // tx
	UCSR0B |= (1<<RXEN0);  //rx
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0C |= (1<<USBS0);
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


void uart_transmit_char(char c) {
	while(!(UCSR0A & (1<<UDRE0)));  // wait for empty transmit buffer
	UDR0 = c;                              // transmit character
}

void uart_transmit_string(char *s) {
	uint8_t i = 0;
	for (i = 0; i < strlen(s); i++) {
		uart_transmit_char(s[i]);
	}
}

void print_num(uint16_t digits) {
	char buffer[50];
	sprintf(buffer, "%u\n", digits);
	uart_transmit_string(buffer);
}

void print_distance(uint16_t distance_cm) {
	char buffer[50];
	sprintf(buffer, "Distance: %u cm\r\n", distance_cm);
	uart_transmit_string(buffer);
}

void print_adc(uint16_t adc, uint16_t duty_cycle) {
	char buffer[50];
	sprintf(buffer, "ADC: %u, Duty Cycle: %u%% \n", adc, duty_cycle);
	uart_transmit_string(buffer);
}


char String[25];

ISR(TIMER4_CAPT_vect) {

	if (capture_count == 0) {
		start_time = ICR4;
		//print_num(start_time);
		capture_count = 1;
		TCCR4B &= ~(1 << ICES4); // Set to capture falling edge
		} else {
		end_time = ICR4;
		//print_num(end_time);
		capture_count = 0;
		TCCR4B |= (1 << ICES4); // Set to capture rising edge

		// Measure pulse width - convert to us
		uint16_t period = end_time - start_time ;
		if (period < 0) {
			period += 65535;
	}

		period = period / 2; // convert to us

		// Convert pulse width to distance
		distance_cm = period / 58; // div by 58 for cm
		print_distance(distance_cm);
		
	}


	//TIFR1 |= (1 << ICF1);
}

// void adc_init() {
// 	/////ADC SETUP///////
// 	
// 	//Clear power reduction
// 	PRR0 &= ~(1<<PRADC);
// 	// Select Vref = AVcc
// 	ADMUX |= (1<<REFS0);
// 	ADMUX &= ~(1<<REFS1);
// 	//128 prescaler, 125kHz
// 	ADCSRA |= (1<<ADPS0);
// 	ADCSRA |= (1<<ADPS1);
// 	ADCSRA |= (1<<ADPS2);
// 	//channel 0
// 	ADMUX &= ~(1<<MUX0);
// 	ADMUX &= ~(1<<MUX1);
// 	ADMUX &= ~(1<<MUX2);
// 	ADMUX &= ~(1<<MUX3);
// 	//auto trigger
// 	ADCSRA |= (1<<ADATE);
// 	//free running
// 	ADCSRB &= ~(1<<ADTS0);
// 	ADCSRB &= ~(1<<ADTS1);
// 	ADCSRB &= ~(1<<ADTS2);
// 	//Disable digital input buffer
// 	DIDR0 |= (1<<ADC0D);
// 	//Enable ADC
// 	ADCSRA |= (1<<ADEN);
// 
// 	//Start conversion
// 	ADCSRA |= (1<<ADSC);
// 	
// 	//Enable interrupt
// 	ADCSRA |= (1<<ADIE);
// }
void ultrasonic_init() {
	// Set up echo pin as input, ICP1 Timer 1 Capture Input
	DDRL &= ~(1<<DDL0);
	DDRD &= ~(1<<DDD4);

	// Set up trig pin as output
	DDRB |= (1<<TRIG_PIN);

	// timer1 setup - prescale 8, normal mode
	TCCR4B &= ~(1 << CS40); // prescale 8
	TCCR4B |= (1 << CS41);
	TCCR4B &= ~(1 << CS42);

	TCCR4A &= ~(1 << WGM40); // normal
	TCCR4A &= ~(1 << WGM41);
	TCCR4B &= ~(1 << WGM42);
	TCCR4B &= ~(1 << WGM43);

	// set up capture
	
	
	//TIFR1 |= (1 << ICF1);
	//TCCR1B |= (1<<ICNC1);
	//looks for rising edge
	TCCR4B |= (1 << ICES4);
	
	//tim1 input capture enabled
	TIMSK4 |= (1 << ICIE4);
	TCCR4A = 0;
	TCNT4 = 0;

	//sei(); // Enable interrupts
}

void servo_init() {
	
}






int main(void) 
{
	// initialize all
	
	uart_init();
	
	//adc_init();
	ultrasonic_init();
	sei();
	

	while (1) {
		// Send trigger pulse
		PORTB |= (1 << TRIG_PIN);
		_delay_us(10);
		PORTB &= ~(1 << TRIG_PIN);
		_delay_us(60);


		// print_num(OCR0A);
// 		sprintf(String,"ADC: %u\n", TCNT1);
// 		UART_putstring(String);
// 		sprintf(String,"test");
// 		UART_putstring(String);
	}
}

