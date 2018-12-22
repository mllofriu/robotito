PROG=i2cblink

all: twi.o $(PROG).c
	avr-c++ -std=c++11 -O1 -mmcu=attiny24a -o $(PROG).elf $(PROG).c twi.o
	avr-objcopy -j .text -j .data --change-section-lma .eeprom=0 -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t24 -P /dev/ttyACM12 -c avrisp -b 19200 -U flash:w:$(PROG).hex

twi.o: TinyWire/twi.cpp TinyWire/twi.h
	avr-c++ -std=c++11 -O1 -mmcu=attiny24a TinyWire/twi.cpp -c -o twi.o
