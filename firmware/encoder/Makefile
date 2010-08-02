CC=avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=atmega325
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS)
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref	
OBJ2HEX=avr-objcopy 



SERIAL = mk000001



####### Files:

SOURCES       = mk.c \
				button.c

OBJECTS	      = mk.o \
				button.o

TARGET=	mk

####### Implicit rules

.SUFFIXES: .c .o .cpp .cc .cxx .C


.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

%.elf:	$(OBJECTS)


####### Build rules

$(TARGET):	$(OBJECTS)
		$(CC) $(CFLAGS) -o $(TARGET) $(OBJECTS)
		$(CC) $(ALL_CFLAGS) $(OBJECTS) --output $(TARGET).elf $(LDFLAGS)	
		$(OBJ2HEX) -R .eeprom -O ihex $(TARGET) $(TARGET).hex
program: $(TARGET).hex
	avarice -2 --erase --program --file $(TARGET).hex --jtag usb --jtag-bitrate 500KHz --write-fuses ff99ff

debug: $(TARGET).hex
	avarice --jtag /dev/tty.usbserial-1B1 --jtag-bitrate 500KHz localhost:8086

erase:
	avrdude -p m325 -b 115200 -P usb -c avrispmkII -e 
	#avarice --erase --jtag /dev/tty.usbserial-1B1 --jtag-bitrate 500KHz

fuses: 
	avrdude -p m325 -b 115200 -P usb -c avrispmkII -U lfuse:w:0xff:m
	#avarice --write-fuses fc89ff --jtag /dev/tty.usbserial-1B1 --jtag-bitrate 500KHz

pro: $(TARGET).hex
	avrdude -p m325 -b 115200 -P usb -c avrispmkII -e -D -U flash:w:$(TARGET).hex:i -U lfuse:w:0xff:m
	#avarice --erase --program --file $(TARGET).hex --jtag /dev/tty.usbserial-1B1 --jtag-bitrate 500KHz --write-fuses fc89ef

qpro: $(TARGET).hex
	avrdude -p m325 -b 115200 -P usb -c avrispmkII -e -V -D -U flash:w:$(TARGET).hex:i

p: $(TARGET).hex
	avrdude -p m325 -P usb -c avrispmkII -U lfuse:w:0xff:m -U hfuse:w:0x99:m -U efuse:w:0xff:m -U flash:w:$(TARGET).hex:i


boot: $(TARGET).hex
	avrdude -p m325 -b 115200 -P /dev/tty.usbserial-$(SERIAL) -c arduino -e -V -D -U flash:w:$(TARGET).hex:i


####### Compile

mk.o:		mk.c
button.o:	button.c
