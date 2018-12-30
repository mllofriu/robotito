PROG=i2cblink

all: usi_i2c_slave.o $(PROG).c
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -o $(PROG).elf $(PROG).c usi_i2c_slave.o
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(PROG).hex

usi_i2c_slave.o: usi_i2c_slave.c
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c usi_i2c_slave.c -o usi_i2c_slave.o
