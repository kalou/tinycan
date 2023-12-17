CC=avr-gcc
# Flags for attiny85
CFLAGS=-Wall -g -Os -mmcu=attiny85 -fverbose-asm -DDEBUG
LDFLAGS=-mmcu=attiny85
# Adapt to your programmer type & device info:
DUDEFLAGS=-cusbtiny -pt85
OBJS=avr.o can.o

%.hex: %
	avr-objcopy -R .eeprom -O ihex $< $@

all: avr.hex

avr: $(OBJS)

test_can: CC=gcc
test_can: CFLAGS=-DDEBUG
test_can: LDFLAGS=
test_can: test_can.o can.o

test: test_can
	./test_can

avr-install: avr.hex
	avrdude $(DUDEFLAGS) -U flash:w:$<

clean:
	rm -f avr avr.hex $(OBJS) avr.elf test_can.o test_can
