all:
	avr-gcc -O1 -mmcu=attiny861 -o drivemotor.elf drivemotor.c
	avr-objcopy -j .text -j .data -O ihex drivemotor.elf drivemotor.hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:drivemotor.hex
