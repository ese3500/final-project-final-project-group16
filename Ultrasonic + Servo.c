/*
 * GccApplication13.c
 *
 * Created: 4/7/2023 2:58:58 PM
 * Author : Opham
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>



#define TRIG_PIN DDB1
#define ECHO_PIN DDB0

volatile uint16_t capture_count = 0;
volatile uint16_t start_time = 0;
volatile uint16_t end_time = 0;
volatile uint16_t distance_cm = 0;
volatile uint8_t is_continous = 1;
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

void set_OCR0A_dist(uint16_t inp) {
	uint16_t temp = ((double) 0.612 * inp) + 58.775;
	// print_num(temp);
	OCR0A = temp;
}

ISR(TIMER1_CAPT_vect) {
	//uart_transmit_string("here\n");
	if (capture_count == 0) {
		start_time = ICR1;
		//print_num(start_time);
		capture_count = 1;
		TCCR1B &= ~(1 << ICES1); // Set to capture falling edge
		} else {
		end_time = ICR1;
		//print_num(end_time);
		capture_count = 0;
		TCCR1B |= (1 << ICES1); // Set to capture rising edge

		// Measure pulse width - convert to us
		uint16_t pw = end_time - start_time ;
		if (pw < 0) {
			pw += 65535;
		}

		pw = pw / 2; // convert to us

		// Convert pulse width to distance
		distance_cm = pw / 58; // div by 58 for cm
		print_distance(distance_cm);
	}

	TIFR1 |= (1 << ICF1);
}

void ultrasonic_init() {
	// Set up echo pin as input
	DDRB &= ~(1 << ECHO_PIN);

	// Set up trig pin as output
	DDRB |= (1 << TRIG_PIN);

	// timer1 setup - prescale 8, normal mode
	TCCR1B &= ~(1 << CS10); // prescale 8
	TCCR1B |= (1 << CS11);
	TCCR1B &= ~(1 << CS12);

	TCCR1A &= ~(1 << WGM10); // normal
	TCCR1A &= ~(1 << WGM11);
	TCCR1A &= ~(1 << WGM12);
	TCCR1B &= ~(1 << WGM10);

	// set up capture
	TIMSK1 |= (1 << ICIE1);
	TCCR1B |= (1 << ICES1);
	TIFR1 |= (1 << ICF1);
	TCCR1B |= (1<<ICNC1);

	sei(); // Enable interrupts
}






int main() {

	// initialize all
	ultrasonic_init();
	uart_init();

	while (1) {
		// Send trigger pulse
		PORTB |= (1 << TRIG_PIN);
		_delay_us(10);
		PORTB &= ~(1 << TRIG_PIN);
		_delay_us(60);

		// button toggles frequency mode

		// print_num(OCR0A);
	}
}

//QUESTION 4 - PWM
//#define TOP 35 // (F_CPU / (4 * PRESCALER * 440))
//
//int main(void) {
//    // Set up PD6 (pin 6) as output
//
//    cli();
//
//    DDRD |= (1 << DDD6);
//
//    TCCR0B |= (1 << WGM02);
//    TCCR0A &= ~(1 << WGM01);
//    TCCR0A |= (1 << WGM00); // set Phase Correct PWM mode 5
//
//
//    TCCR0B |= (1 << CS02);  // 256 prescale
//    TCCR0B &= ~(1 << CS01);
//    TCCR0B &= ~(1 << CS00);
//    OCR0A = TOP; // set TOP
//
//    TCCR0A &= ~(1 << COM0A1); // set compare output mode toggle
//    TCCR0A |= (1 << COM0A0);
//
//    sei();
//
//    while (1) {
//
//    }
//}

// QUESTION 3 - CTC Mode
//#define OCR0A_VALUE 70 // (16000000 / (2 * 256 * 440)) - 1
//
//int main(void) {
//    // Set up PD6 (pin 6) as output
//    DDRD |= (1 << DDD6);
//
//
//    TCCR0B |= (0 << WGM02);
//    TCCR0A |= (1 << WGM01) | (0 << WGM00); // CTC
//    TCCR0B |= (1 << CS02) | (0 << CS01) | (0 << CS00); // 256 prescale
//    OCR0A = OCR0A_VALUE;
//
//    TCCR0A |= (0 << COM0A1) | (1 << COM0A0); // toggle buzzer
//
//    while (1) {
//    }
//}

// QUESTION 2 - Normal Mode
//#define OCR0A_VALUE 70 // (F_CPU / (2 * PRESCALER * 440)) - 1
//
//ISR(TIMER0_COMPA_vect) {
//    // Toggle PD6 output
//    PORTD ^= (1 << PORTD6);
//    TCNT0 = 0;
//}
//
//int main(void) {
//    // Set up PD6 (pin 6) as output
//    DDRD |= (1 << DDD6);
//
//    TCCR0A &= ~(1 << WGM00) & ~(1 << WGM01) & ~(1 << WGM02);  // normal mode
//    TCCR0B |= (1 << CS02); // no prescale
//    TCCR0B &= ~(1 << CS01) & ~(1 << CS00);
//    OCR0A = OCR0A_VALUE;                      // set compare match value
//
//    TCCR0A &= ~(1 << COM0A1); // set compare output mode toggle
//    TCCR0A |= (1 << COM0A0);
//
//    TIMSK0 |= (1 << OCIE0A);    // enable compare match interrupt
//    sei();  // enable global interrupts
//
//    while (1) {
//        // Main program loop
//    }
//}


//SETUP
//volatile uint8_t overflow_count = 0;
//
//ISR(TIMER0_OVF_vect) {
//    //overflow_count++;
//    if (overflow_count >= 125) {  // toggle output at around 1 kHz (test)
//        PORTD ^= (1 << PORTD6);
//        overflow_count = 0;
//    }
//    uart_transmit_string("here");
//    PORTD ^= (1 << PORTD6);        // toggle PD6 output
//    overflow_count = 0;
//    TCNT0 = 0;
//}
//
//int main(void) {
//
//    DDRD |= (1 << DDD6);
//
//
//
//    TCCR0B &= ~(1 << CS02); // no prescale
//    TCCR0B &= ~(1 << CS01);
//    TCCR0B &= ~(1 << CS00);
//
//    TCCR0A &= ~(1 << WGM00); // normal
//    TCCR0A &= ~(1 << WGM01);
//    TCCR0B &= ~(1 << WGM02);
//
//    TIMSK0 |= (1 << TOIE0);               // enable overflow interrupt
//    TIFR0 |= (1 << TOV0);
//
//    sei();                                // enable global interrupts
//
//    while (1) {
//        // Main program loop
//    }
//}
