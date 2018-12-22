PROG=i2cblink

all: $(PROG).c TinyWireS/utility/USI_TWI_Slave.c
	avr-gcc -std=c++11 -O1 -mmcu=attiny24a -o $(PROG).elf $(PROG).c TinyWireS/utility/USI_TWI_Slave.c
	avr-objcopy -j .text -j .data --change-section-lma .eeprom=0 -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t24 -P /dev/ttyACM1 -c avrisp -b 19200 -U flash:w:$(PROG).hex


