PROG=stopcond

all: $(PROG).c
	avr-gcc -O1 -mmcu=attiny24a -o $(PROG).elf $(PROG).c 
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t24 -P /dev/ttyACM6 -c avrisp -b 19200 -U flash:w:$(PROG).hex

