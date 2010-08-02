/**********************************************************/
/*                                                        */
/* mk bootloader - a bootloader for monome kits           */
/* brian crabtree (tehn@monome.org)						  */
/* revision: 100724									      */
/* information: http://monome.org                         */
/* source: http://github.com/tehn/mk                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*   Optiboot              http://code.google.com/p/optiboot/ */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/interrupt.h>

// usb pins
#define C0_TXE 0x01
#define C1_RXF 0x02
#define C2_WR 0x04
#define C3_RD 0x08

/* STK500 constants list, from AVRDUDE */
#define STK_OK              0x10
#define STK_FAILED          0x11  // Not used
#define STK_UNKNOWN         0x12  // Not used
#define STK_NODEVICE        0x13  // Not used
#define STK_INSYNC          0x14  // ' '
#define STK_NOSYNC          0x15  // Not used
#define ADC_CHANNEL_ERROR   0x16  // Not used
#define ADC_MEASURE_OK      0x17  // Not used
#define PWM_CHANNEL_ERROR   0x18  // Not used
#define PWM_ADJUST_OK       0x19  // Not used
#define CRC_EOP             0x20  // 'SPACE'
#define STK_GET_SYNC        0x30  // '0'
#define STK_GET_SIGN_ON     0x31  // '1'
#define STK_SET_PARAMETER   0x40  // '@'
#define STK_GET_PARAMETER   0x41  // 'A'
#define STK_SET_DEVICE      0x42  // 'B'
#define STK_SET_DEVICE_EXT  0x45  // 'E'
#define STK_ENTER_PROGMODE  0x50  // 'P'
#define STK_LEAVE_PROGMODE  0x51  // 'Q'
#define STK_CHIP_ERASE      0x52  // 'R'
#define STK_CHECK_AUTOINC   0x53  // 'S'
#define STK_LOAD_ADDRESS    0x55  // 'U'
#define STK_UNIVERSAL       0x56  // 'V'
#define STK_PROG_FLASH      0x60  // '`'
#define STK_PROG_DATA       0x61  // 'a'
#define STK_PROG_FUSE       0x62  // 'b'
#define STK_PROG_LOCK       0x63  // 'c'
#define STK_PROG_PAGE       0x64  // 'd'
#define STK_PROG_FUSE_EXT   0x65  // 'e'
#define STK_READ_FLASH      0x70  // 'p'
#define STK_READ_DATA       0x71  // 'q'
#define STK_READ_FUSE       0x72  // 'r'
#define STK_READ_LOCK       0x73  // 's'
#define STK_READ_PAGE       0x74  // 't'
#define STK_READ_SIGN       0x75  // 'u'
#define STK_READ_OSCCAL     0x76  // 'v'
#define STK_READ_FUSE_EXT   0x77  // 'w'
#define STK_READ_OSCCAL_EXT 0x78  // 'x'

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_4S     (_BV(WDE3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDE3) | _BV(WDE0) | _BV(WDE))

/* Function Prototypes */
/* The main function is in init9, which removes the interrupt vector table */
/* we don't need. It is also 'naked', which means the compiler does not    */
/* generate any entry or exit code itself. */
int main(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
void putch(char);
uint8_t getch(void);
static inline void getNch(uint8_t); /* "static inline" is a compiler hint to reduce code size */
void verifySpace();
uint8_t getLen();
static inline void watchdogReset();
void watchdogConfig(uint8_t x);
void appStart() __attribute__ ((naked));

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(0x100))
#define address (*(uint16_t*)(0x200))
#define length  (*(uint8_t*)(0x202))

/* main program starts here */
int main(void) {
	cli();
	SP=RAMEND;
	asm volatile ("clr __zero_reg__");

	uint8_t ch;
	ch = MCUSR;
	MCUSR = 0;
	if (!(ch & _BV(EXTRF))) appStart();

	watchdogConfig(WATCHDOG_OFF);

	DDRD = 0;
	PORTD = 0;
	PORTC = 0;
	DDRC = C2_WR | C3_RD;

	while(1) {
		ch = getch();
		
		if(ch == STK_GET_PARAMETER) {
			// GET PARAMETER returns a generic 0x03 reply - enough to keep Avrdude happy
			getNch(1);
			putch(0x03);
		}
		else if(ch == STK_SET_DEVICE) {
			// SET DEVICE is ignored
			getNch(20);
		}
		else if(ch == STK_SET_DEVICE_EXT) {
			// SET DEVICE EXT is ignored
			getNch(5);
		}
		else if(ch == STK_LOAD_ADDRESS) {
			// LOAD ADDRESS
			address = getch();
			address = (address & 0xff) | (getch() << 8);
			address += address; // Convert from word address to byte address
			verifySpace();
		}
		else if(ch == STK_UNIVERSAL) {
			// UNIVERSAL command is ignored
			getNch(4);
			putch(0x00);
		}
		// Write memory, length is big endian and is in bytes  
	    else if(ch == STK_PROG_PAGE) {
	      // PROGRAM PAGE - we support flash programming only, not EEPROM
	      uint8_t *bufPtr;
	      uint16_t addrPtr;

	      getLen();

	      // Immediately start page erase - this will 4.5ms
	      boot_page_erase((uint16_t)(void*)address);

	      // While that is going on, read in page contents
	      bufPtr = buff;
	      do *bufPtr++ = getch();
	      while (--length);

	      // Read command terminator, start reply
	      verifySpace();

	      // If only a partial page is to be programmed, the erase might not be complete.
	      // So check that here
	      boot_spm_busy_wait();

	      // Copy buffer into programming buffer
	      bufPtr = buff;
	      addrPtr = (uint16_t)(void*)address;
	      ch = SPM_PAGESIZE / 2;
	      do {
	        uint16_t a;
	        a = *bufPtr++;
	        a |= (*bufPtr++) << 8;
	        boot_page_fill((uint16_t)(void*)addrPtr,a);
	        addrPtr += 2;
	      } while (--ch);

	      // Write from programming buffer
	      boot_page_write((uint16_t)(void*)address);
	      boot_spm_busy_wait();

	      // Reenable read access to flash
	      boot_rww_enable();
	    }
// Read memory block mode, length is big endian.  
		else if(ch == STK_READ_PAGE) {
			// READ PAGE - we only read flash
			getLen();
			verifySpace();

			do putch(pgm_read_byte_near(address++));
			while (--length);
		}

// Get device signature bytes  
		else if(ch == STK_READ_SIGN) {
			// READ SIGN - return what Avrdude wants to hear
			verifySpace();
			putch(0x1E);
			putch(0x95);
			putch(0x05);
		}
		else if (ch == 'Q') {
			// Adaboot no-wait mod
	//		watchdogConfig(WATCHDOG_16MS);
			verifySpace();
		}
		else {
			// This covers the response to commands like STK_ENTER_PROGMODE
			verifySpace();
		}
		
		putch(STK_OK);
	}

}

// ****************************************** mk rewrite ***************
void putch(char cc) {
	DDRD = 0xFF;		// setup PORTD for output
	while(PINC & C0_TXE);	// wait for buffer
	PORTC |= C2_WR;     
	PORTD = cc;
	PORTC &= ~(C2_WR);
}

// ****************************************** mk rewrite ***************
uint8_t getch(void) {
	uint8_t c;

//  watchdogReset();

	PORTD = 0;              // setup PORTD for input
	DDRD = 0;               // input w/ tristate	
	while((PINC & C1_RXF));	// wait for data
	PORTC |= C3_RD;
	c = PIND;
	PORTC &= ~(C3_RD);

	return c;
}


void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
	if (getch() != CRC_EOP) appStart();
  	putch(STK_INSYNC);
}


uint8_t getLen() {
  getch();
  length = getch();
  return getch();
}

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

// changed WDTCSR to WDTCR for atmega325
void watchdogConfig(uint8_t x) {
  	WDTCR = _BV(WDCE) | _BV(WDE);
	WDTCR = x;
}

void appStart() {
 // watchdogConfig(WATCHDOG_OFF);
  __asm__ __volatile__ (
    // Jump to RST vector
    "clr r30\n"
    "clr r31\n"
    "ijmp\n"
  );
}
