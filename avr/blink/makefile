all:
	avr-gcc -O1 -mmcu=attiny861 -o blink.elf blink.c
	avr-objcopy -j .text -j .data -O ihex blink.elf blink.hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:blink.hex
