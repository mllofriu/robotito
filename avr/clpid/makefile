all:
	avr-gcc -O1 -mmcu=attiny861 -o clpid.elf clpid.c
	avr-objcopy -j .text -j .data -O ihex clpid.elf clpid.hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:clpid.hex

fuses:
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m




