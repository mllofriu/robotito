PROG=i2cblink

all: twi.o $(PROG).c
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -o $(PROG).elf $(PROG).c twi.o
	avr-objcopy -j .text -j .data --change-section-lma .eeprom=0 -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(PROG).hex

twi.o: TinyWire/twi.cpp TinyWire/twi.h
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 TinyWire/twi.cpp -c -o twi.o

fuses:
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

