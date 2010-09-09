/************************************************************************

version: encoders alternate (grid rotation for 40h keypad compatability)
aux-version: 0

************************************************************************/

#define F_CPU 16000000UL
#include <inttypes.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "button.h"

// firmware version: encoders
#define FW_VERSION 0

// protocol
#define NUM_TYPES 15

#define KEYDOWN 0
#define KEYUP 1
#define LEDON 2
#define LEDOFF 3
#define LEDROW1 4
#define LEDCOL1 5
#define LEDROW2 6
#define LEDCOL2 7
#define FRAME 8
#define CLEAR 9
#define INTENSITY 10
#define MODE 11
#define GRIDS 12
#define AUXIN 13
#define AUXOUT 14

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
#define OUTPUT_BUFFER_LENGTH 200
#define KEY_REFRESH_RATE 25
#define AUX_REFRESH_RATE 5
#define RX_STARVE 20

// eeprom locations
#define EEPROM_NUM_GRIDS 0
#define EEPROM_PORT_ENABLE 1


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
uint8_t i1, i2, i3, i4;
uint8_t n1, n2, n3, n4;
volatile uint8_t output_buffer[OUTPUT_BUFFER_LENGTH];
volatile uint8_t output_write;
volatile uint8_t output_read;
volatile uint8_t num_grids, port_enable;


// aux (encoder) globals
char map[4][4] = { {0,1,-1,0}, {-1,0,0,1}, {1,0,0,-1}, {0,-1,1,0} };
volatile uint8_t enc_now[8], enc_prev[8];
volatile char enc_delta[8];




// KEYPAD SCAN INT
// ===============================================================
// ===============================================================
ISR(TIMER0_COMP_vect)
{	
	for(i1=0;i1<8;i1++) {
		// sel row
		i2 = i1 << 4;				// to prevent momentary flicker to row 0
		PORTB = i2;
		
		button_last[i1] = button_current[i1];
		button_last[i1+8] = button_current[i1+8];
		button_last[i1+16] = button_current[i1+16];
		button_last[i1+24] = button_current[i1+24];
		
		_delay_us(2);				// wait for voltage fall! due to high resistance pullup
		PORTE |= (E7_LD);	
		_delay_us(1);

		for(i2=0;i2<8;i2++) {
			
			// =================================================
			if(num_grids > 0) {
				i3 = i1;
				i4 = (PINB & B3_SER1)!=0;
			
				if (!i4) 
	                button_current[i3] |= (1 << i2);
	            else
	                button_current[i3] &= ~(1 << i2);

				buttonCheck(i3, i2);

				if (button_event[i3] & (1 << i2)) {
	                button_event[i3] &= ~(1 << i2);	

					output_buffer[output_write] = i4 << 4;
					//output_buffer[output_write+1] = ((7-i1)<<4) | i2;
					output_buffer[output_write+1] = ((7-i2)<<4) | (7-i1);
					output_write = (output_write + 2) % OUTPUT_BUFFER_LENGTH;
				}
			}
			
			// =================================================
			if(num_grids > 1) {
				i3 = i1 + 8;
				i4 = (PINB & B2_SER2)!=0;
			
				if (!i4) 
	                button_current[i3] |= (1 << i2);
	            else
	                button_current[i3] &= ~(1 << i2);

				buttonCheck(i3, i2);

				if (button_event[i3] & (1 << i2)) {
	                button_event[i3] &= ~(1 << i2);	

					output_buffer[output_write] = i4 << 4;
					//output_buffer[output_write+1] = ((15-i1)<<4) | i2;
					output_buffer[output_write+1] = ((i1+8)<<4) | (7-i2);
					output_write = (output_write + 2) % OUTPUT_BUFFER_LENGTH;
				}
			}
			
			// =================================================
			if(num_grids > 2) {
				i3 = i1 + 16;
				i4 = (PINB & B1_SER3)!=0;
			
				if (!i4) 
	                button_current[i3] |= (1 << i2);
	            else
	                button_current[i3] &= ~(1 << i2);

				buttonCheck(i3, i2);

				if (button_event[i3] & (1 << i2)) {
	                button_event[i3] &= ~(1 << i2);	

					output_buffer[output_write] = i4 << 4;
					output_buffer[output_write+1] = ((7-i1)<<4) | (i2+8);
					output_write = (output_write + 2) % OUTPUT_BUFFER_LENGTH;
				}
			}
			
			// =================================================
			if(num_grids > 3) {
				i3 = i1 + 24;
				i4 = (PINB & B0_SER4)!=0;
			
				if (!i4) 
	                button_current[i3] |= (1 << i2);
	            else
	                button_current[i3] &= ~(1 << i2);

				buttonCheck(i3, i2);

				if (button_event[i3] & (1 << i2)) {
	                button_event[i3] &= ~(1 << i2);	

					output_buffer[output_write] = i4 << 4;
					//output_buffer[output_write+1] = ((15-i1)<<4) | (i2+8);
					output_buffer[output_write+1] = ((i2+8)<<4) | (i1+8);
					output_write = (output_write + 2) % OUTPUT_BUFFER_LENGTH;
				}
			}
			
			PORTE |= (E6_CLK);
			_delay_us(1);
			PORTE &= ~(E6_CLK);		
			_delay_us(1);
		}

		PORTE &= ~(E7_LD);	
	}
	
	TCNT0 = 0;
}

// AUX INT
// ===============================================================
// ===============================================================
ISR(TIMER1_COMPA_vect)
{
	if(port_enable) {
		n1 = PINA;
		enc_now[0] = n1 & 0x03;
		enc_delta[0] += map[enc_prev[0]][enc_now[0]];
		enc_prev[0] = enc_now[0];
	
		enc_now[1] = (n1 & 0x0C)>>2;
		enc_delta[1] += map[enc_prev[1]][enc_now[1]];
		enc_prev[1] = enc_now[1];
	
		enc_now[2] = (n1 & 0x30)>>4;
		enc_delta[2] += map[enc_prev[2]][enc_now[2]];
		enc_prev[2] = enc_now[2];
	
		enc_now[3] = (n1 & 0xC0)>>6;
		enc_delta[3] += map[enc_prev[3]][enc_now[3]];
		enc_prev[3] = enc_now[3];
	
		n1 = PINF;
		enc_now[4] = n1 & 0x03;
		enc_delta[4] += map[enc_prev[4]][enc_now[4]];
		enc_prev[4] = enc_now[4];
	
		enc_now[5] = (n1 & 0x0C)>>2;
		enc_delta[5] += map[enc_prev[5]][enc_now[5]];
		enc_prev[5] = enc_now[5];
	
		enc_now[6] = (n1 & 0x30)>>4;
		enc_delta[6] += map[enc_prev[6]][enc_now[6]];
		enc_prev[6] = enc_now[6];
	
		enc_now[7] = (n1 & 0xC0)>>6;
		enc_delta[7] += map[enc_prev[7]][enc_now[7]];
		enc_prev[7] = enc_now[7];
	}
					
	TCNT1 = 0;
}


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
		// _delay_us(1);
		PORTE &= ~(E0_CLK);
	}

	for(i=0;i<8;i++) {
		if(data2 & (1<<(7-i))) 
			PORTE |= ALL_SER;
		else
			PORTE &= ~(ALL_SER);

		PORTE |= (E0_CLK);
		// _delay_us(1);
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
//		_delay_us(1);
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
//		_delay_us(1);
		PORTE &= ~(E0_CLK);
	}

	PORTE |= (E1_LD); 
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
	uint8_t rx[10];	// input buffer
	uint8_t usb_state, sleep_state;
	
	uint8_t display[4][8];
	
	char enc[8];
	
	uint8_t update_display;
	
	uint8_t packet_length[NUM_TYPES];
	
	// setup packet length
	packet_length[KEYDOWN] = 2; 
	packet_length[KEYUP] = 2;
	packet_length[LEDON] = 2;
	packet_length[LEDOFF] = 2;
	packet_length[LEDROW1] = 2;
	packet_length[LEDCOL1] = 2;
	packet_length[LEDROW2] = 3;
	packet_length[LEDCOL2] = 3;
	packet_length[FRAME] = 9;
	packet_length[CLEAR] = 1;
	packet_length[INTENSITY] = 1;
	packet_length[MODE] = 1;
	packet_length[GRIDS] = 1;
	packet_length[AUXIN] = 3;
	packet_length[AUXOUT] = 3;
	

	// pin assignments
	DDRE = 0xff;	// all output
	DDRB = B4_A0 | B5_A1 | B6_A2; 
	DDRC = C2_WR | C3_RD;
	DDRD = 0;                                 			

	PORTE = 0;
	PORTB = 0; 
	PORTD = 0;

	// aux pin assignments (encoders)
	DDRA = 0;
	DDRF = 0;
	
	PORTA = 0xff;	// activate internal pullups
	PORTF = 0xff;                          
	
	
	// read EEPROM data
	while(EECR & (1<<EEWE));
	EEAR = EEPROM_NUM_GRIDS;
	EECR |= (1<<EERE);
	num_grids = EEDR;
	
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
	to_all_led(1,1); _delay_ms(64);
	to_all_led(2,2); _delay_ms(64);
	to_all_led(3,4); _delay_ms(64);
	to_all_led(4,8); _delay_ms(64);
	to_all_led(5,16); _delay_ms(64);
	to_all_led(6,32); _delay_ms(64);
	to_all_led(7,64); _delay_ms(64);
	to_all_led(8,128); _delay_ms(64);
	
	

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
	update_display = 0;

	buttonInit();
	output_read = 0;
	output_write = 0;
		
	for(i1=0;i1<8;i1++) {
		enc_delta[i1] = 0;
		enc_now[i1] = 0;
		enc_prev[i1] = 0;
	}
	
		
	// keypad timer init
	TCCR0A |= (1<<CS02) | (1<<CS00); // timer0 on, prescale clk/1024 (p95)
	TIMSK0 |= (1 << OCIE0A);// | (1<< TOIE0);  // enable timer0 interrupts
	OCR0A = KEY_REFRESH_RATE;
	
	// aux timer init
	TCCR1A = 0;
	TCCR1B |= (1<<CS12);// | (1<<CS10); // clk/256
	TIMSK1 |= (1 << OCIE1A);
	OCR1A = AUX_REFRESH_RATE;
	
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
			TIMSK1 |= (1 << OCIE1A);			
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
					rx_type = rx[0] >> 4;
					if(rx_type < NUM_TYPES) {	
						rx_length = packet_length[rx_type];
						rx_count++;
						rx_timeout = 0;
					}
				}
				else rx_count++;


				if(rx_count == rx_length) {
					rx_count = 0;
					
					if(rx_type == GRIDS) {
						while(EECR & (1<<EEWE));
						num_grids = rx[0] & 0x0f;
						EEAR = EEPROM_NUM_GRIDS;
						EEDR = num_grids;
						EECR |= (1<<EEMWE);
						EECR |= (1<<EEWE);
					}
					else if(rx_type == AUXIN) {
						i1 = rx[0] & 0x0f;
						if(i1==0) {			// report version
							output_buffer[output_write] = (AUXOUT << 4);
							output_buffer[output_write+1] = FW_VERSION;
							output_buffer[output_write+2] = 0;
							output_write = (output_write + 3) % OUTPUT_BUFFER_LENGTH;
						}
						else if(i1==1) {	// set port enables, write eeprom
							while(EECR & (1<<EEWE));
							port_enable = rx[1];
							EEAR = EEPROM_PORT_ENABLE;
							EEDR = port_enable;
							EECR |= (1<<EEMWE);
							EECR |= (1<<EEWE);
						}
					}
					else if(rx_type == INTENSITY) {
						i1 = rx[0] & 0x0f;
						to_all_led(10,i1);
					}
					else if(rx_type == MODE) {
						i1 = rx[0] & 0x03;
						
						if(i1 == 0) {							// NORMAL
							to_all_led(12, 1);					// shutdown mode off
							to_all_led(15, 0);					// test mode off
						}
						else if(i1 == 1) {						
							to_all_led(12, 1);					// shutdown mode off
							to_all_led(15, 1);					// test mode on
						}
						else if(i1 == 2) {
							to_all_led(12, 0);					// shutdown mode on
							to_all_led(15, 0);					// test mode off
						}
					}
					else if(rx_type == LEDCOL1) {		// ok*************************	
						i1 = rx[0] & 0x0F;
						i2 = i1 > 7;
						i1 = i1 & 0x07;
						
						if(i2==0) {
							i1 = 1 << (7-i1);
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if((rev[rx[1]]) & i4) display[i2][i3] |= i1;
								else display[i2][i3] &= ~i1;												
							}
						}
						else if(i2==1) display[i2][i1] = rev[rx[1]];
						
						update_display++;
					}
					else if(rx_type == LEDCOL2) {		// ok*************************	
						i1 = rx[0] & 0x0F;
						i2 = i1 > 7;
						i1 = i1 & 0x07;
						
						if(i2==0) {
							display[i2+2][7-i1] = rx[2];
					
							i1 = 1 << (7-i1);
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if((rev[rx[1]]) & i4) display[i2][i3] |= i1;
								else display[i2][i3] &= ~i1;												
							}
						}
						else if(i2==1) {
							display[i2][i1] = rev[rx[1]];
							
							i1 = 1 << i1;
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if((rx[2]) & i4) display[i2+2][i3] |= i1;
								else display[i2+2][i3] &= ~i1;												
							}
						}
						
						update_display++;
					}
					else if(rx_type == LEDROW1) {		// ok*************************	
						i2 = (rx[0] & 0x08) >> 2;
						
						if(i2==0) {
							i1 = 7 - (rx[0] & 0x07);
							display[i2][i1] = rev[rx[1]];
						}
						else if(i2==2) {
							i1 = 1 << (rx[0] & 0x07);
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rev[rx[1]] & i4) display[i2][i3] |= i1;
								else display[i2][i3] &= ~i1;												
							}
						}
						
						update_display++;
					}
					else if(rx_type == LEDROW2) {		// ok*************************		
						i2 = (rx[0] & 0x08) >> 2;
						
						if(i2==0) {
							i1 = 7 - (rx[0] & 0x07);
							display[i2][i1] = rev[rx[1]];
							
							i1 = 1 << (7-(rx[0] & 0x07));
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rx[2] & i4) display[i2+1][i3] |= i1;
								else display[i2+1][i3] &= ~i1;												
							}
							
						}
						else if(i2==2) {
							i1 = rx[0] & 0x07;
							display[i2+1][i1] = rx[2];
							
							i1 = 1 << (rx[0] & 0x07);
							for(i3=0;i3<8;i3++) {
								i4 = 1 << i3;
								if(rev[rx[1]] & i4) display[i2][i3] |= i1;
								else display[i2][i3] &= ~i1;												
							}
						}
						
						update_display++;
					}
					else if(rx_type == FRAME) {
						i1 = rx[0] & 0x03;
						
						if(i1==0) {
							for(i2=0;i2<8;i2++) {
								display[i1][7-i2] = rev[rx[i2+1]];
							}
						}
						else if(i1==1) {
							for(i2=0;i2<8;i2++) {
								i4 = 1 << (7-i2);
								for(i3=0;i3<8;i3++) {
									if(rx[i2+1] & (1 << i3)) display[i1][i3] |= i4;
									else display[i1][i3] &= ~i4;												
								}
							}
						}
						else if(i1==2) {
							for(i2=0;i2<8;i2++) {
								i4 = 1 << i2;
								for(i3=0;i3<8;i3++) {
									if(rev[rx[i2+1]] & (1 << i3)) display[i1][i3] |= i4;
									else display[i1][i3] &= ~i4;												
								}
							}
						}
						else if(i1==3) {
							for(i2=0;i2<8;i2++) {
								display[i1][i2] = rx[i2+1];
							}
						}
						
						update_display++;
					}
					else if(rx_type == LEDON) {		// ok*************************
						i1 = rx[1] & 0x07;
						i2 = 7-((rx[1] >> 4) & 0x07);
						i3 = ((rx[1] & 0x80) >> 7) + ((rx[1] & 0x08) >> 2);
									
						// 0 flip a/b, reverse a
						// 1 reverse a, reverse b
						// 2 ok!
						// 3 flip a/b, reverse b
						
						if(i3==0) display[i3][7-i1] |= (1<<i2);
						else if(i3==1) display[i3][7-i2] |= (1<<(7-i1));
						else if(i3==2) display[i3][i2] |= (1<<i1);
						else if(i3==3) display[i3][i1] |= (1<<(7-i2));
						
						update_display++;
					}
					else if(rx_type == LEDOFF) {	// ok*************************
						i1 = rx[1] & 0x07;
						i2 = 7-((rx[1] >> 4) & 0x07);
						i3 = ((rx[1] & 0x80) >> 7) + ((rx[1] & 0x08) >> 2);
						
						if(i3==0) display[i3][7-i1] &= ~(1<<i2);
						else if(i3==1) display[i3][7-i2] &= ~(1<<(7-i1));
						else if(i3==2) display[i3][i2] &= ~(1<<i1);
						else if(i3==3) display[i3][i1] &= ~(1<<(7-i2));
						
						update_display++;
					}
					else if(rx_type == CLEAR) {
						i1 = rx[0] & 0x01;
						if(i1) i1 = 255;
						
						for(i2=0;i2<8;i2++) {
							display[0][i2] = i1;
							display[1][i2] = i1;
							display[2][i2] = i1;
							display[3][i2] = i1;
						}
				
						update_display++;
					}
				}

				PORTC |= C3_RD;
				_delay_us(1);			// wait for valid data
			}
			
			if(update_display) {
				cli();
				update_display = 0;
				
				for(i1=0;i1<8;i1++) {
					to_led(i1+1,display[0][i1],display[1][i1],display[2][i1],display[3][i1]);
				}
				sei();
			}

			// ====================== check encoder deltas
			
			cli();
			for(i1=0;i1<8;i1++) {
				enc[i1] = enc_delta[i1];
				enc_delta[i1] &= 3;
			}	
			sei();
			
			for(i1=0;i1<8;i1++) {
				enc[i1] >>= 2;
				if(enc[i1] && (port_enable & (1 << i1))) {
					output_buffer[output_write] = (AUXOUT << 4) + 1;
					output_buffer[output_write+1] = i1;
					output_buffer[output_write+2] = enc[i1];
					output_write = (output_write + 3) % OUTPUT_BUFFER_LENGTH;
				}
			}

			// ====================== check/send output data
			
			cli();
			PORTD = 0;                      // setup PORTD for output
			DDRD = 0xFF;

			while(output_read != output_write) {
				
				PORTC |= C2_WR;
				PORTD = output_buffer[output_read];
				//_delay_us(1);
				PORTC &= ~(C2_WR);
				
				output_read = (output_read + 1) % OUTPUT_BUFFER_LENGTH;
			}
			sei();
			
			// ====================== check usb/sleep status
			if(PINC & C4_PWREN) {
				sleep_state = 1;
				TIMSK0 = 0; // turn off keypad checking int
				TIMSK1 = 0;

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
