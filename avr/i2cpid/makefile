PROG=i2cpid

all: $(PROG).c TinyWireS.o TinyWireS.a
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -o $(PROG).elf $(PROG).c TinyWireS.a
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U flash:w:$(PROG).hex

TinyWireS.a: TinyWireS.o USI_TWI_Slave.o
	avr-ar rcs TinyWireS.a TinyWireS.o USI_TWI_Slave.o

TinyWireS.o: TinyWireS/TinyWireS.cpp  
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c TinyWireS/TinyWireS.cpp 

USI_TWI_Slave.o: TinyWireS/utility/USI_TWI_Slave.c
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c TinyWireS/utility/USI_TWI_Slave.c

clean:
	rm -rf *.o *.hex *.a

fuses:
	avrdude -p t861 -P /dev/ttyACM0 -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

