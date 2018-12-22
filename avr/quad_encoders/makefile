PROG=quadenc

all: $(PROG).c
	avr-c++ -std=c++11 -O1 -mmcu=attiny24a -o $(PROG).elf $(PROG).c
	avr-objcopy -j .text -j .data --change-section-lma .eeprom=0 -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t24 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(PROG).hex
