all:
	avr-gcc -O1 -mmcu=attiny24 -o blink.elf blink.c
	avr-objcopy -j .text -j .data -O ihex blink.elf blink.hex
	avrdude -F -p t24 -P /dev/ttyACM1 -c avrisp -b 19200 -U flash:w:blink.hex
