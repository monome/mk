/************************************************************************

version: encoders
aux-version: 0

*************************************************************************

cmds:

to device

GRIDS (1 byte): (12<<4) & x 
	where x is 0-4 (number of grids active)
osc /sys/grids x

		
AUXIN (3 bytes): (13<<4) & x, y, z
	where x = 0: query request, ask device for query return (firmware version)
osc /sys/aux/version
	where x = 1: enable/disable ports, y and z = ports, 8bit each
osc /sys/aux/enable y z
	where x = 2: set port direction, y and z = ports, 8bit each
osc /sys/aux/direction y z
	where x = 3: set digital output states, y and z = ports, 8 bit each
osc /sys/aux/state y z
	

	
from device:

AUXOUT (3 bytes): (14<<4) & x, y, z
	where x = 0: query return, y = version
osc /sys/aux/version y
	where x = 3: analog, y = port, z = value
osc /sys/aux/analog y z
	where x = 2: digital, y = number, z = state
osc /sys/aux/digital y z
	where x = 1: encoder, y = encoder number, z = change (two's complement, signed char)
osc /sys/aux/encoder y z (z is signed char)


adc - PORTF
io - PORTA
*/

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
#define OUTPUT_BUFFER_LENGTH 80
#define KEY_REFRESH_RATE 25
#define AUX_REFRESH_RATE 5
#define RX_STARVE 20

// eeprom locations
#define EEPROM_NUM_GRIDS 0
#define EEPROM_PORT_ENABLE 1


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
		
		_delay_us(5);				// wait for voltage fall! due to high resistance pullup
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
					output_buffer[output_write+1] = ((7-i1)<<4) | i2;
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
					output_buffer[output_write+1] = ((15-i1)<<4) | i2;
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
					output_buffer[output_write+1] = ((15-i1)<<4) | (i2+8);
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
		_delay_us(1);
		PORTE &= ~(E0_CLK);
	}

	for(i=0;i<8;i++) {
		if(data2 & (1<<(7-i))) 
			PORTE |= ALL_SER;
		else
			PORTE &= ~(ALL_SER);

		PORTE |= (E0_CLK);
		_delay_us(1);
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
		_delay_us(1);
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
		_delay_us(1);
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
					else if(rx_type == LEDCOL1) {			
						i1 = rx[0] & 0x0F;
						i2 = i1 > 7;
						i1 = i1 & 0x07;
					
						display[i2][i1] = rx[1];
						
						to_led(i1+1,display[0][i1],display[1][i1],display[2][i1],display[3][i1]);
					}
					else if(rx_type == LEDCOL2) {			
						i1 = rx[0] & 0x0F;
						i2 = i1 > 7;
						i1 = i1 & 0x07;
					
						display[i2][i1] = rx[1];
						display[i2+2][i1] = rx[2];
					
						to_led(i1+1,display[0][i1],display[1][i1],display[2][i1],display[3][i1]);
					}
					else if(rx_type == LEDROW1) {			
						i1 = 1 << (rx[0] & 0x07);
						i2 = (rx[0] & 0x08) >> 2;
						
						for(i3=0;i3<8;i3++) {
							i4 = 1 << i3;
							// OPTIMIZE??
							if(rx[1] & i4) display[i2][i3] |= i1;
							else display[i2][i3] &= ~i1;												
						}
						
						for(i3=0;i3<8;i3++)
							to_led(i3+1,display[0][i3],display[1][i3],display[2][i3],display[3][i3]);
					}
					else if(rx_type == LEDROW2) {			
						i1 = 1 << (rx[0] & 0x07);
						i2 = (rx[0] & 0x08) >> 2;
						
						for(i3=0;i3<8;i3++) {
							i4 = 1 << i3;
							// OPTIMIZE??
							if(rx[1] & i4) display[i2][i3] |= i1;
							else display[i2][i3] &= ~i1;
							
							if(rx[2] & i4) display[i2+1][i3] |= i1;
							else display[i2+1][i3] &= ~i1;							
						}
						
						for(i3=0;i3<8;i3++)
							to_led(i3+1,display[0][i3],display[1][i3],display[2][i3],display[3][i3]);
					}
					else if(rx_type == FRAME) {
						i1 = rx[0] & 0x03;
						
						for(i2=0;i2<8;i2++) {
							i4 = 1 << i2;
							for(i3=0;i3<8;i3++) {
								if(rx[i2+1] & (1 << i3)) display[i1][i3] |= i4;
								else display[i1][i3] &= ~i4;												
							}
						}
						
						for(i3=0;i3<8;i3++)
							to_led(i3+1,display[0][i3],display[1][i3],display[2][i3],display[3][i3]);
					}
					else if(rx_type == LEDON) {
						i1 = rx[1] & 0x07;
						i2 = 7-((rx[1] >> 4) & 0x07);
						i3 = ((rx[1] & 0x80) >> 7) + ((rx[1] & 0x08) >> 2);
						
						display[i3][i2] |= (1<<i1);
						
						to_led(i2+1,display[0][i2],display[1][i2],display[2][i2],display[3][i2]);					
					}
					else if(rx_type == LEDOFF) {
						i1 = rx[1] & 0x07;
						i2 = 7-((rx[1] >> 4) & 0x07);
						i3 = ((rx[1] & 0x80) >> 7) + ((rx[1] & 0x08) >> 2);
						
						display[i3][i2] &= ~(1<<i1);
						
						to_led(i2+1,display[0][i2],display[1][i2],display[2][i2],display[3][i2]);					
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
				
						for(i3=0;i3<8;i3++)
							to_led(i3+1,display[0][i3],display[1][i3],display[2][i3],display[3][i3]);					
					}
				}

				PORTC |= C3_RD;
				_delay_us(1);			// wait for valid data
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
			
			PORTD = 0;                      // setup PORTD for output
			DDRD = 0xFF;

			while(output_read != output_write) {
				
				PORTC |= C2_WR;
				PORTD = output_buffer[output_read];
				//_delay_us(1);
				PORTC &= ~(C2_WR);
				
				output_read = (output_read + 1) % OUTPUT_BUFFER_LENGTH;
			}
			
			
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
