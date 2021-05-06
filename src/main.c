/*
 * Author: Vasil Vodenicharov
 */
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUDRATE 115200

/* Format
 Byte 1: Sync-Value (0xA5, constant)
 Byte 2: Sync-Value (0x5A, constant)
 Byte 3: Version Info (0x02, constant, OpenEEG version P2 format)
 Byte 4: Packet counter (0x00 - 0xff, sequentially increases with every packet)
 Byte 5: Channel 0 value high byte 0/4
 Byte 6: Channel 0 value low byte  0/5
 Byte 7: Channel 1 value high byte 1/6
 Byte 8: Channel 1 value low byte  1/7
 Byte 9: Channel 2 value high byte 2/8
 Byte 10: Channel 2 value low byte 2/9
 Byte 11: Channel 3 value high byte
 Byte 12: Channel 3 value low byte
 Byte 13: Channel 4 value high byte
 Byte 14: Channel 4 value low byte
 Byte 15: Channel 5 value high byte
 Byte 16: Channel 5 value low byte
 Byte 17: Value of PIND (GPIO input pin states of Port D)
 *
 */

volatile uint8_t package[17] = { 0xA5, 0x5A, 0x02, 0 };
volatile uint8_t package_counter = 1;
volatile uint8_t channel = 0;

void init_UART(uint32_t baud) {

	uint16_t br = F_CPU / 8 / baud - 1;		//Set BitRate
	UBRR0H = (uint8_t) (br >> 8);			//Load High BR byte
	UBRR0L = (uint8_t) (br & 0xff);			//Load Low BR byte

	UCSR0A |= (1 << U2X0);					//Enable 2x speed
	UCSR0B |= (1 << TXEN0);					//Enable Transmit
	//UCSR0B |= (1 << RXCIE0); 				//Enable UART receive interrupts
}

void init_ADC() {
	ADMUX |= (1 << REFS0); 			//Set reference to AVCC (5V);
	ADCSRA |= (1 << ADPS2) + (1 << ADPS1) + (1 << ADPS0); //set Freq to 1/128 * CPU freq
	ADCSRA |= (1 << ADEN); 			//enable ADC
	ADCSRA |= (1 << ADIF);			//Clear interrupt status
	ADCSRA |= (1 << ADIE); 			//Enable ADC complete interrupt

}

//Interrupt routines
/* ISR (USART_RX_vect) { 			//UART Read interrupt for Debug purposes

	char buff = UDR0;				//Read UART input
	if ((buff == 'a') || (buff == 'A')) {
		ADCSRA |= (1 << ADSC);		//Start a single conversion
	}
} */

ISR (ADC_vect) {		//ADC complete interrupt

	if (channel == 6) {
		channel = 0;
		package[16] = PIND;				//write PIND to byte 17
		UCSR0B |= (1 << UDRIE0);		//Enable USART_UDRE interrupts
	} else {
		package[5 + (2 * channel)] = ADCL;
		package[4 + (2 * channel)] = ADCH;
		ADMUX &= ~(0x07);
		ADMUX |= ++channel;				//Select next channel
		ADCSRA |= (1 << ADSC); 			//Initiate next channel ADC-on
	}

}

ISR (USART_UDRE_vect) { //UART write interrupt

	static int txindex = 0; 		//Transmit buffer index
	UDR0 = package[txindex++];
	if (txindex == 17) {
		package[3] = package_counter++;
		UCSR0B &= ~(1 << UDRIE0);
		txindex = 0;
	}

}

ISR(INT0_vect) { 		//PD2 interrupt

	//Enable : TCCR0B|=0x04;//(1-5); //depending on prescaler 4->1/256; ->timer1 -> ~1s
	TCCR1B ^= (1 << 1); //(1<<2)		//prescaler
	//TCCR0B^=(1<<2);
}

ISR(TIMER1_OVF_vect) {	//Timer1 overflow interrupt
	//TCNT0=reload;
	ADCSRA |= (1 << ADSC);
	TCNT1 = (0xFFFF-20000);//625
}

void init_TIMER() {
	TCNT1 = (0xFFFF-20000);//reload value; 625
	//TIMSK0|=(1<<0); //Enable timer0 overflow interrupt
	TIMSK1 |= (1 << 0);
}

void init_GPIO() {
	PORTD |= (1 << 2); 		// activate pull-up resistor on PD2
	EICRA |= (1 << ISC01); 	// set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0); 	// enable INT0
}

int main(void) {
	init_GPIO();
	init_TIMER();
	init_UART(BAUDRATE);			//Initiate UART with specified BAUDRATE
	init_ADC();						//Initiate ADC and enable interrupts
	sei();							//Enable global interrupts
	while (1)
		;
}
