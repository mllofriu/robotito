PROG=i2cblink

all: usi_i2c_slave.o $(PROG).c
	avr-gcc -O1 -mmcu=attiny24a -o $(PROG).elf $(PROG).c usi_i2c_slave.o
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t24 -P /dev/ttyACM12 -c avrisp -b 19200 -U flash:w:$(PROG).hex

usi_i2c_slave.o: usi_i2c_slave.c
	avr-gcc -O1 -mmcu=attiny24a -c usi_i2c_slave.c -o usi_i2c_slave.o
