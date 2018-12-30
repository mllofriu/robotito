PROG=i2cblink

all: $(PROG).c TinyWireS/utility/USI_TWI_Slave.c
	avr-gcc -O1 -mmcu=attiny861 -o $(PROG).elf $(PROG).c TinyWireS/utility/USI_TWI_Slave.c
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(PROG).hex

fuses:
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

