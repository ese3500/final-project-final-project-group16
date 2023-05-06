#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define TRIG_PIN DDH3

volatile uint16_t capture_count = 0;
volatile uint16_t start_time = 0;
volatile uint16_t overflow = 0;
volatile uint16_t end_time = 0;
volatile uint16_t distance_cm = 0;
volatile uint16_t totalDistance = 0;
volatile uint16_t averageDistance = 0;
volatile uint16_t tempindex = 0;
volatile uint8_t is_continous = 1;
volatile int count = 0;
volatile int aim = 0;
const double PI =  3.1415926;
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((16000000UL / (BAUD_RATE * 16UL))) - 1)

/////////////////////////////UART///////////////////////////////////

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
//////////////////////////////////////////////////////////////////////////

char String[25];
int distanceArray[10];







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
		if (tempindex < 9) {
			tempindex++;
		} else {
			tempindex = 0;
		}
		distanceArray[tempindex] = distance_cm;
		//print_distance(distance_cm);
		
	}

}

void adc_init() {
	/////ADC SETUP///////
	DDRB &= ~(1 << DDB4);//joystick press
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	
	ADMUX |= (1<<REFS0);       //Set Voltage reference to Avcc (5v)
	
	ADCSRA |= (1<<ADEN);       //Turn on ADC
	
	ADCSRA |= (1<<ADSC);      //Do an initial conversion

}

int getAngle (double distance) {
	double initialDistance = 1.8; //max distance it can shoot at 45 degrees;
	
	if (distance <= initialDistance) {
		double initialVelocity = sqrt(initialDistance * 9.81);
		double angle = 0.5 * asin(9.81 * distance/(initialVelocity * initialVelocity)) * 180 / PI; //solve for angle
		double ocrFraction = (90-angle)/90;
		int newAngle = (int) (ocrFraction * (124-62)); // converts angle into ocr value
// 		sprintf(String,"Angle: %u\n", (int) angle);
// 		UART_putstring(String);
		return (62+newAngle); 

	} 
}

//Returns the ADC value of the chosen channel
uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xE0;           //Clear bits MUX0-4
	ADMUX |= channel&0x07;   //Defines the new ADC channel to be read by setting bits MUX0-2
	ADCSRB = channel&(1<<3); //Set MUX5
	ADCSRA |= (1<<ADSC);      //Starts a new conversion
	while(ADCSRA & (1<<ADSC));  //Wait until the conversion is done
	return ADCW;
}         

void ultrasonic_init() {
	// Set up echo pin as input, ICP4 Timer 4 Capture Input
	DDRL &= ~(1<<DDL0);

	// Set up trig pin as output
	DDRH |= (1<<TRIG_PIN);

	// timer4 setup - prescale 8, normal mode
	TCCR4B &= ~(1 << CS40); // prescale 8
	TCCR4B |= (1 << CS41);
	TCCR4B &= ~(1 << CS42);

	TCCR4A &= ~(1 << WGM40); // normal
	TCCR4A &= ~(1 << WGM41);
	TCCR4B &= ~(1 << WGM42);
	TCCR4B &= ~(1 << WGM43);

	//noise reduction
	TCCR4B |= (1<<ICNC4);
	
	//looks for rising edge
	TCCR4B |= (1 << ICES4);
	
	//tim1 input capture enabled
	TIMSK4 |= (1 << ICIE4);
	TCCR4A = 0;
	TCNT4 = 0;

	//sei(); // Enable interrupts
}

//up down
void servo_init() {
	DDRB |= (1 << DDB5);
		
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
		
	OCR1A = 63;
	ICR1 = 1249;
}

//pan
void servo2_init() {
	DDRL |= (1 << DDL3);
	
	//Clear on Compare Match
	TCCR5A |= (1<<COM5A1);
	TCCR5A &= ~(1<<COM5A0);
	
	//256 prescaling
	TCCR5B |= (1<<CS52);
	TCCR5B &= ~(1<<CS51);
	TCCR5B &= ~(1<<CS50);
	
	//Mode 14 fast pwm
	TCCR5A &= ~(1<<WGM50);
	TCCR5A |= (1<<WGM51);
	TCCR5B |= (1<<WGM52);
	TCCR5B |= (1<<WGM53);
	
	OCR5A = 93;
	ICR5 = 1249;
}

void motor_init() {
		DDRE |= (1 << DDE3);
		
		//Clear on Compare Match
		TCCR3A |= (1<<COM3A1);
		TCCR3A &= ~(1<<COM3A0);
		
		//256 prescaling
		TCCR3B |= (1<<CS32);
		TCCR3B &= ~(1<<CS31);
		TCCR3B &= ~(1<<CS30);
		
		//Mode 14 fast pwm
		TCCR3A &= ~(1<<WGM30);
		TCCR3A |= (1<<WGM31);
		TCCR3B |= (1<<WGM32);
		TCCR3B |= (1<<WGM33);

		ICR3 = 255;
		OCR3A = 255;
}





int main(void) 
{
	// initialize all
	
	uart_init();
	adc_init();
	servo_init();
	servo2_init();
	motor_init();
	ultrasonic_init();
	sei();
	

	while (1) {
		// Send trigger pulse
		PORTH |= (1 << TRIG_PIN);
		_delay_us(10);
		PORTH &= ~(1 << TRIG_PIN);
		_delay_us(60);
		sprintf(String,"distance: %u\n", averageDistance);
		UART_putstring(String);
		
		if ((read_adc(000000) < 300) & (OCR1A < 125)) {
			OCR1A++;
			} else if ((read_adc(000000) > 700) & (OCR1A > 62)) {
			OCR1A--;
			} 
		if ((read_adc(000001) < 300) & (OCR5A > 62)) {
			OCR5A--;
			} else if ((read_adc(000001) > 700) & (OCR5A < 125)) {
			OCR5A++;

			//OCR1A = getAngle((double) averageDistance / 100);
			} 
			
		
//  		if (PINB & (1<<PINB4) && (aim == 0)) {
//  			aim = 1;
// 			sprintf(String,"YES \n");
// 			UART_putstring(String);
//  			OCR1A = getAngle((double) distance_cm / 100);
//  		} else if (PINB & (1<<PINB4) && (aim == 1)) {
//  			aim = 0;
//  		}
		totalDistance = 0;
		for (int i = 0; i<10; i++) {
			totalDistance += distanceArray[i];
		}
		averageDistance = (int)(totalDistance / 10);
		


		
		// print_num(OCR0A);

		sprintf(String,"OCR5A: %u\n", OCR5A);
		UART_putstring(String);
// 		sprintf(String,"OCR1A: %u\n", OCR1A);
// 		UART_putstring(String);

 		//sprintf(String,"test");
 		//UART_putstring(String);
	}
}

