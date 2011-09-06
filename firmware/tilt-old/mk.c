/************************************************************************
version: tilt-old
*************************************************************************
*/

#define F_CPU 16000000UL
#include <inttypes.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include "button.h"


#define SIZE_X 16
#define SIZE_Y 16
#define GRIDS 4


// firmware version: tilt
#define FW_VERSION 2

// eeprom location
#define EEPROM_PORT_ENABLE 0

// protocol incoming
#define _SYS_QUERY 0x00
#define _SYS_QUERY_ID 0x01
#define _SYS_WRITE_ID 0x02
#define _SYS_GET_GRID_OFFSET 0x03
#define _SYS_SET_GRID_OFFSET 0x04
#define _SYS_GET_GRID_SIZE 0x05
#define _SYS_SET_GRID_SIZE 0x06
#define _SYS_SCAN_ADDR 0x07
#define _SYS_SET_ADDR 0x08
#define _SYS_QUERY_VERSION 0x0F

#define _LED_SET0 0x10
#define _LED_SET1 0x11
#define _LED_ALL0 0x12
#define _LED_ALL1 0x13
#define _LED_MAP 0x14
#define _LED_ROW 0x15
#define _LED_COL 0x16
#define _LED_INT 0x17

#define _TILT_GET_STATE 0x80
#define _TILT_SET_STATE_ON 0x81
#define _TILT_SET_STATE_OFF 0x82


const uint8_t packet_length[256] = {
	1,1,33,1,4,1,3,1,3,0,0,0,0,0,0,1,
	3,3,1,1,11,4,4,2,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	1,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

// protocol outgoing
#define _SYS_QUERY_RESPONSE 0x00
#define _SYS_ID 0x01
#define _SYS_REPORT_GRID_OFFSET 0x02
#define _SYS_REPORT_GRID_SIZE 0x03
#define _SYS_FOUND_ADDR 0x04
#define _SYS_REPORT_VERSION 0x05


// led pins
#define E0_CLK 0x01
#define E1_LD 0x02   
#define E2_SER4 0x04 
#define E3_SER3 0x08 
#define E4_SER2 0x10 
#define E5_SER1 0x20 
#define ALL_SER 0x3C

// key in pins
#define E6_CLK 0x40
#define E7_LD 0x80
#define B0_SER4 0x01
#define B1_SER3 0x02
#define B2_SER2 0x04
#define B3_SER1 0x08

// key sel pins
#define B4_A0 0x10
#define B5_A1 0x20
#define B6_A2 0x40

// usb pins
#define C0_TXE 0x01
#define C1_RXF 0x02
#define C2_WR 0x04
#define C3_RD 0x08
#define C4_PWREN 0x10
#define B7_USB 0x80

// tuning
#define OUTPUT_BUFFER_LENGTH 256
#define KEY_REFRESH_RATE 15
#define AUX_REFRESH_RATE 100
#define RX_STARVE 20

static const uint8_t rev[] =
{
0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};

// globals
volatile uint8_t port_enable;
volatile uint8_t scan_keypads;

uint8_t output_buffer[OUTPUT_BUFFER_LENGTH];
uint8_t output_write;
uint8_t output_read;

volatile int16_t an[2][2];
volatile int16_t a;
volatile int16_t an_bucket[2][8];
volatile uint8_t an_num;
volatile uint8_t an_index;
volatile int32_t an_accum[2];



// send packet to all led drivers
// ===============================================================
void to_all_led(char data1, char data2)
{
	uint8_t i;

	PORTE &= ~(E1_LD);

	for(i=0;i<8;i++) {
		if(data1 & (1<<(7-i))) 
			PORTE |= ALL_SER;
		else
			PORTE &= ~(ALL_SER);
		
		PORTE |= (E0_CLK);
		PORTE &= ~(E0_CLK);
	}

	for(i=0;i<8;i++) {
		if(data2 & (1<<(7-i))) 
			PORTE |= ALL_SER;
		else
			PORTE &= ~(ALL_SER);

		PORTE |= (E0_CLK);
		PORTE &= ~(E0_CLK);
	}

	PORTE |= (E1_LD); 
}

// update led drivers-- first byte to all, individual second bytes
// ===============================================================
void to_led(char data_all, char data1, char data2, char data3, char data4)
{
	uint8_t i;

	PORTE &= ~(E1_LD);

	for(i=0;i<8;i++) {
		if(data_all & (1<<(7-i))) 
			PORTE |= ALL_SER;
		else
			PORTE &= ~(ALL_SER);
		

		PORTE |= (E0_CLK);
		PORTE &= ~(E0_CLK);
	}

	for(i=0;i<8;i++) {
		if(data1 & (1<<(7-i))) PORTE |= (E5_SER1);
		else PORTE &= ~(E5_SER1);
		
		if(data2 & (1<<(7-i))) PORTE |= (E4_SER2);
		else PORTE &= ~(E4_SER2);
		
		if(data3 & (1<<(7-i))) PORTE |= (E3_SER3);
		else PORTE &= ~(E3_SER3);
		
		if(data4 & (1<<(7-i))) PORTE |= (E2_SER4);
		else PORTE &= ~(E2_SER4);				

		PORTE |= (E0_CLK);
		PORTE &= ~(E0_CLK);
	}

	PORTE |= (E1_LD); 
}

// AUX INT
// ===============================================================
// ===============================================================
ISR(TIMER1_COMPA_vect)
{
	if(port_enable) {
		an[an_num][1] = an[an_num][0];
		an_accum[an_num] -= an_bucket[an_num][an_index];
		a = (ADCW>>2) - 128;
		an_bucket[an_num][an_index] = a;
		an_accum[an_num] += a;
		an[an_num][0] = an_accum[an_num] >> 3;
	
		// send tilt,val via usb
	
		if(an[an_num][0] != an[an_num][1]) {
			output_buffer[output_write] = 0x81;
			output_write++;
			output_buffer[output_write] = 0;
			output_write++;
			output_buffer[output_write] = an[0][0] & 0xff;
			output_write++;
			output_buffer[output_write] = an[0][0] >> 8;
			output_write++;
			output_buffer[output_write] = an[1][0] & 0xff;
			output_write++;
			output_buffer[output_write] = an[1][0] >> 8;
			output_write++;
			output_buffer[output_write] = 0;
			output_write++;
			output_buffer[output_write] = 0;
			output_write++;
		}
	
		if(an_num==1) {
			ADMUX = (1<<MUX0);
			an_num = 0;
			an_index = (an_index + 1) % 8;
		}
		else {
			ADMUX = 0;
			an_num = 1;
		}
	
		ADCSRA |= (1<<ADSC);		// start conversion
	}
	
	TCNT1 = 0;
}


// KEYPAD SCAN INT
// ===============================================================
// ===============================================================
ISR(TIMER0_COMP_vect)
{
	scan_keypads = 1;
	TCNT0 = 0;
}

// main
// ===============================================================
// ===============================================================
// ===============================================================
int main(void)
{
	uint8_t i1,i2,i3,i4;
	uint8_t starve;
	uint8_t rx_count;
	uint8_t rx_length;
	uint8_t rx_type;
	uint8_t rx_timeout;
	uint8_t rx[66];	// input buffer
	uint8_t usb_state, sleep_state;
	uint8_t update_display;
	uint8_t display[4][8];
	
	char id[32];
	
	uint8_t keypad_row;
		

	// pin assignments
	DDRE = 0xff;	// all output
	DDRB = B4_A0 | B5_A1 | B6_A2; 
	DDRC = C2_WR | C3_RD;
	DDRD = 0;                                 			

	PORTE = 0;
	PORTB = 0; 
	PORTD = 0;

	// aux pin assignments
	DDRA = 0;
	DDRF = 0;
	
	PORTA = 0;//xff;	// activate internal pullups
	PORTF = 0;//xff;                          
	
	for(i1=0;i1<32;i1++) id[i1]=0;
	strcpy(id,"mk");
	
	// read eeprom for tilt activation
	while(EECR & (1<<EEWE));
	EEAR = EEPROM_PORT_ENABLE;
	EECR |= (1<<EERE);
	port_enable = EEDR;	

	// init led drivers
	to_all_led(11, 7);                                    	// set scan limit to full range
	to_all_led(10, 15);                               		// set to max intensity
	for(i1 = 1; i1 < 9; i1++) to_all_led(i1, 0);         	// clear pattern 
	
	to_all_led(12, 1);                                    	// come out of shutdown mode 
	to_all_led(15, 0);                                    	// test mode off

	// startup sequence
	to_led(8,12,0,0,0);
	_delay_ms(64);
	to_all_led(10, 13);
	to_led(8,18,0,0,0); to_led(7,12,0,0,0);
	_delay_ms(64);
	to_all_led(10, 11);
	to_led(8,33,0,0,0); to_led(7,18,0,0,0); to_led(6,12,0,0,0);
	_delay_ms(64);
	to_all_led(10, 9);
	to_led(8,64,0,0,0); to_led(7,64,0,0,0); to_led(6,33,0,0,0); to_led(5,30,0,0,0);
	_delay_ms(64);
	to_all_led(10, 7);
	to_led(8,0,0,1,0); to_led(7,0,0,1,0); to_led(6,0,0,1,0); to_led(5,128,0,0,0); to_led(4,64,0,0,0); to_led(3,33,0,0,0); to_led(2,30,0,0,0);
	_delay_ms(64);
	to_all_led(10, 5);
	to_led(8,0,192,16,0); to_led(7,0,56,16,0); to_led(6,0,7,8,0); to_led(5,0,0,8,0); to_led(4,0,0,4,0); to_led(3,0,0,4,0); to_led(2,0,0,2,0); to_led(1,0,0,1,0);
	_delay_ms(64);
	to_all_led(10, 3);
	to_led(8,0,0,128,32); to_led(7,0,0,128,16); to_led(6,0,0,128,16); to_led(5,0,0,128,8); to_led(4,0,0,64,6); to_led(3,0,128,64,1); to_led(2,0,112,64,0); to_led(1,0,15,32,0);
	_delay_ms(64);
	to_all_led(10, 1);
	to_led(8,0,0,0,0); to_led(7,0,0,0,0); to_led(6,0,0,0,0); to_led(5,0,0,0,0); to_led(4,0,0,0,128); to_led(3,0,0,0,128); to_led(2,0,0,0,64); to_led(1,0,0,0,48);
	_delay_ms(64);
	

	// end startup sequence
	for(i1 = 1; i1 < 9; i1++) to_all_led(i1, 0);			// clear pattern
	to_all_led(10, 15);                            			// set to max intensity

	// init data
	for(i1=0;i1<10;i1++) rx[i1] = 0;
			
	for(i1=0;i1<4;i1++)
		for(i2=0;i2<8;i2++)
			display[i1][i2] = 0;

	i1 = i2 = i3 = i4 = 0;
	rx_count = rx_type = rx_timeout = 0;
	rx_length = 1;
	usb_state = 0;
	sleep_state = 1;
	update_display=0;
	keypad_row = 0;
	output_read = 0;
	output_write = 0;
	

	buttonInit();
	
	
	// init ADC
	//ADMUX = (1<<ADLAR);	// set left align (for 8 bit mode)
	ADMUX = (1<<MUX0);
	ADCSRA = (1<<ADEN) | (1<<ADPS2);// | (1<<ADATE);	// turn on ADC, prescale
	//DIDR0 = 0x03;	// disable digital inputs on ADC0-1
	ADCSRA |= (1<<ADSC);		// start conversion
	an[0][0] = an[0][1] = an[1][0] = an[1][1] = 127;
	for(i1=0;i1<8;i1++) an_bucket[0][i1] = an_bucket[1][i1] = 127;
	an_accum[0] = an_accum[1] = 1016; // 127 * 8
	an_num = 0;
	an_index = 0;
	
	port_enable = 255;
	
	// aux timer init
	TCCR1A = 0;
	TCCR1B |= (1<<CS12);// | (1<<CS10); // clk/256
	TIMSK1 |= (1 << OCIE1A);
	OCR1A = AUX_REFRESH_RATE;
		
		
	// keypad timer init
	TCCR0A |= (1<<CS02) | (1<<CS00); // timer0 on, prescale clk/1024 (p95)
	TIMSK0 |= (1 << OCIE0A);// | (1<< TOIE0);  // enable timer0 interrupts
	OCR0A = KEY_REFRESH_RATE;
	
	// enable ints
	sei();
	

	// main loop
	while(1) {
		// ========================= ASLEEP:
		if(sleep_state) {	 			
			if(!(PINC & C4_PWREN)) {
				sleep_state = 0;				

				to_all_led(12, 1);		// out of shutdown
				for(i1=0;i1<64;i1++) {	// fade in
					to_all_led(10, (i1)/4);
					_delay_ms(8);
				}
			} 

			_delay_ms(255);
			
			TIMSK0 |= (1 << OCIE0A);// | (1<< TOIE0);  // enable timer0 interrupts
		}
		
		// ========================== NORMAL:
		else {
			// ====================== check/read incoming serial	
			PORTD = 0;                  // setup PORTD for input
			DDRD = 0;                   // input w/ tristate

			if(rx_timeout > 40 ) {
				rx_count = 0;
			}
			else rx_timeout++;

			starve = 0;
			
			while((PINC & C1_RXF) == 0 && starve < RX_STARVE) {
				starve++;				// make sure we process keypad data...
										// if we process more input bytes than RX_STARVE
										// we'll jump to sending out waiting keypad bytes
										// and then continue
				PORTC &= ~(C3_RD);
				_delay_us(1);			// wait for valid data
				rx[rx_count] = PIND;
				
				if(rx_count == 0) {		// get packet length if reading first byte
					rx_type = rx[0];
					if(packet_length[rx_type]) {
						rx_length = packet_length[rx_type];
						rx_count++;
						rx_timeout = 0;
					}
				}
				else rx_count++;

				if(rx_count == rx_length) {
					rx_count = 0;
					rx_length = 0;
					
					if(rx_type == _SYS_QUERY) {
						output_buffer[output_write] = _SYS_QUERY_RESPONSE;
						output_write++;
						output_buffer[output_write] = 1;
						output_write++;
						output_buffer[output_write] = 4;
						output_write++;
						
						output_buffer[output_write] = _SYS_QUERY_RESPONSE;
						output_write++;
						output_buffer[output_write] = 2;
						output_write++;
						output_buffer[output_write] = 4;
						output_write++;
						
					}
					else if(rx_type == _SYS_QUERY_ID) {
						output_buffer[output_write] = _SYS_ID;
						output_write++;

						for(i1=0;i1<32;i1++) {
							output_buffer[output_write] = id[i1];
							output_write++;
						}
					}
					else if(rx_type == _SYS_GET_GRID_SIZE) {
						output_buffer[output_write] = _SYS_REPORT_GRID_SIZE;
						output_write++;
						output_buffer[output_write] = SIZE_X;
						output_write++;
						output_buffer[output_write] = SIZE_Y;
						output_write++;
					}
					
					
					else if(rx_type == _TILT_SET_STATE_ON) {
						port_enable = 255;
					}
					else if(rx_type == _TILT_SET_STATE_OFF) {
						port_enable = 0;
					}
					
					
					
					else if(rx_type == _LED_SET0) {
						// _LED_SET0 //////////////////////////////////////////////

						
						i1 = (rx[1] >> 3) + ((rx[2] >> 3)*2); 
						i2 = 7-(rx[1] & 0x07);
						i3 = rx[2] & 0x07;
						
						
						// display[i1][i2] &= ~(1<<i3);
						if(i1==0) display[i1][7-i3] &= ~(1<<i2);
						else if(i1==1) display[i1][7-i2] &= ~(1<<(7-i3));
						else if(i1==2) display[i1][i2] &= ~(1<<i3);
						else if(i1==3) display[i1][i3] &= ~(1<<(7-i2));

						update_display++;
					}
					else if(rx_type == _LED_SET1) {
						// _LED_SET1 //////////////////////////////////////////////

						i1 = (rx[1] >> 3) + ((rx[2] >> 3)*2); 
						i2 = 7-(rx[1] & 0x07);
						i3 = rx[2] & 0x07;
						
						// display[i1][i2] |= (1<<i3);
						if(i1==0) display[i1][7-i3] |= (1<<i2);
						else if(i1==1) display[i1][7-i2] |= (1<<(7-i3));
						else if(i1==2) display[i1][i2] |= (1<<i3);
						else if(i1==3) display[i1][i3] |= (1<<(7-i2));

						update_display++;
					}if(rx_type == _LED_ALL0) {
						// _LED_ALL0 //////////////////////////////////////////////
						for(i1=0;i1<4;i1++) {
							for(i2=0;i2<8;i2++) {
								display[i1][i2] = 0;
							}
						}
						update_display++;
					} else if(rx_type == _LED_ALL1) {
						// _LED_ALL1 //////////////////////////////////////////////
						for(i1=0;i1<4;i1++) {
							for(i2=0;i2<8;i2++) {
								display[i1][i2] = 255;
							}
						}
						update_display++;
					} else if(rx_type == _LED_MAP) {
						// _LED_MAP ///////////////////////////////////////////////
						i1 = (rx[1] >> 3) + (rx[2] >> 3)*2;

						if(i1==0) {
							for(i2=0;i2<8;i2++) {
								display[i1][7-i2] = rev[rx[i2+3]];
							}
						}
						else if(i1==1) {
							for(i2=0;i2<8;i2++) {
								i4 = 1 << (7-i2);
								for(i3=0;i3<8;i3++) {
									if(rx[i2+3] & (1 << i3)) display[i1][i3] |= i4;
									else display[i1][i3] &= ~i4;												
								}
							}
						}
						else if(i1==2) {
							for(i2=0;i2<8;i2++) {
								i4 = 1 << i2;
								for(i3=0;i3<8;i3++) {
									if(rev[rx[i2+3]] & (1 << i3)) display[i1][i3] |= i4;
									else display[i1][i3] &= ~i4;												
								}
							}
						}
						else if(i1==3) {
							for(i2=0;i2<8;i2++) {
								display[i1][i2] = rx[i2+3];
							}
						}
						
						
						
						update_display++;
					} else if(rx_type == _LED_COL) {
						// _LED_COL ///////////////////////////////////////////////
						// x offset is rx[1]
						i1 = (rx[1] >> 3) + (rx[2] >> 3)*2;
						i2 = rx[1] & 0x07;
						
						if(i1==0) {
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rev[rx[3]] & i4) display[i1][i3] |= (1<<(7-i2));
								else display[i1][i3] &= ~(1<<(7-i2));												
							}
						} else if(i1==1) {
							display[i1][i2] = rev[rx[3]];
						} else if(i1==2) { 
							display[i1][7-i2] = rx[3];
						} else if(i1==3) {
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rx[3] & i4) display[i1][i3] |= (1<<i2);
								else display[i1][i3] &= ~(1<<i2);												
							}
						}
						
						update_display++;
					} else if(rx_type == _LED_ROW) {
						// _LED_ROW ///////////////////////////////////////////////
						// y offset is rx[2]
						i1 = (rx[1] >> 3) + (rx[2] >> 3)*2;						
						i2 = rx[2] & 0x07;
						
						if(i1==0) {
							display[i1][7-i2] = rev[rx[3]];
						} else if(i1==1) {
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rx[3] & i4) display[i1][i3] |= (1<<(7-i2));
								else display[i1][i3] &= ~(1<<(7-i2));												
							}
						} else if(i1==2) { 
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rev[rx[3]] & i4) display[i1][i3] |= (1<<i2);
								else display[i1][i3] &= ~(1<<i2);												
							}
						} else if(i1==3) {
							display[i1][i2] = rx[3];
						}

						update_display++;
					} else if(rx_type == _LED_INT) {
						// _LED_INT ///////////////////////////////////////////////
						i1 = rx[1] & 0x0f;
						to_all_led(10,i1);
					}
				}

				PORTC |= C3_RD;
			}
			
			if(update_display) {
				update_display = 0;
				for(i1=0;i1<8;i1++) {
					to_led(i1+1,display[0][i1],display[1][i1],display[2][i1],display[3][i1]);
				}
			}

			// ====================== scan keypads =========================================
			if(scan_keypads) {
				scan_keypads = 0;
				
				PORTD = 0;                      // setup PORTD for output
				DDRD = 0xFF;


				button_last[keypad_row] = button_current[keypad_row];
				button_last[keypad_row+8] = button_current[keypad_row+8];
				button_last[keypad_row+16] = button_current[keypad_row+16];
				button_last[keypad_row+24] = button_current[keypad_row+24];

				_delay_us(2);				// wait for voltage fall! due to high resistance pullup
				PORTE |= (E7_LD);	
				_delay_us(1);

				for(i2=0;i2<8;i2++) {
					// =================================================
					if(GRIDS > 0) {
						i4 = (PINB & B3_SER1)!=0;

						if (!i4) 
			                button_current[keypad_row] |= (1 << i2);
			            else
			                button_current[keypad_row] &= ~(1 << i2);

						buttonCheck(keypad_row, i2);

						if (button_event[keypad_row] & (1 << i2)) {	
			                button_event[keypad_row] &= ~(1 << i2);	

							output_buffer[output_write] = !i4 + 32;
							output_write++;
							output_buffer[output_write] = 7-i2;
							output_write++;
							output_buffer[output_write] = 7-keypad_row;
							output_write++;
						}
					}

					// =================================================
					if(GRIDS > 1) {
						i3 = keypad_row + 8;
						i4 = (PINB & B2_SER2)!=0;

						if (!i4) 
			                button_current[i3] |= (1 << i2);
			            else
			                button_current[i3] &= ~(1 << i2);

						buttonCheck(i3, i2);

						if (button_event[i3] & (1 << i2)) {
			                button_event[i3] &= ~(1 << i2);	

							output_buffer[output_write] = !i4 + 32;
							output_write++;
							output_buffer[output_write] = keypad_row + 8;
							output_write++;
							output_buffer[output_write] = 7-i2;
							output_write++;
						}
					}

					// =================================================
					if(GRIDS > 2) {
						i3 = keypad_row + 16;
						i4 = (PINB & B1_SER3)!=0;

						if (!i4) 
			                button_current[i3] |= (1 << i2);
			            else
			                button_current[i3] &= ~(1 << i2);

						buttonCheck(i3, i2);

						if (button_event[i3] & (1 << i2)) {
			                button_event[i3] &= ~(1 << i2);	
	
							output_buffer[output_write] = !i4 + 32;
							output_write++;
							output_buffer[output_write] = 7-keypad_row;
							output_write++;
							output_buffer[output_write] = i2 + 8;
							output_write++;
						}
					}

					// =================================================
					if(GRIDS > 3) {
						i3 = keypad_row + 24;
						i4 = (PINB & B0_SER4)!=0;

						if (!i4) 
			                button_current[i3] |= (1 << i2);
			            else
			                button_current[i3] &= ~(1 << i2);

						buttonCheck(i3, i2);

						if (button_event[i3] & (1 << i2)) {
			                button_event[i3] &= ~(1 << i2);	

							output_buffer[output_write] = !i4 + 32;
							output_write++;
							output_buffer[output_write] = i2 + 8;
							output_write++;
							output_buffer[output_write] = keypad_row + 8;
							output_write++;
						}
					}


					PORTE |= (E6_CLK);
					PORTE &= ~(E6_CLK);		
				}

				PORTE &= ~(E7_LD);	
				
				keypad_row++;
				keypad_row %= 8;
				PORTB = keypad_row << 4;
			}
			
			
			// ====================== check/send output data
			
			PORTD = 0;                      // setup PORTD for output
			DDRD = 0xFF;

			while(output_read != output_write) {
				PORTC |= C2_WR;
				PORTD = output_buffer[output_read];
				PORTC &= ~(C2_WR);
				output_read++;// = (output_read + 1) % OUTPUT_BUFFER_LENGTH;
			}
			

			
			// ====================== check usb/sleep status
			if(PINC & C4_PWREN) {
				sleep_state = 1;
				TIMSK0 = 0; // turn off keypad checking int

				for(i1=0;i1<16;i1++) {	// fade out
					to_all_led(10, 15-i1);
					_delay_ms(64);
				}
				to_all_led(12, 0);		// shutdown
			} 
		} 		
	}

	return 0;
}
