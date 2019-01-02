PROG=i2cpid
DEV=/dev/ttyACM1

all: $(PROG).c TinyWireS.o TinyWireS.a motor_controller.o
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -o $(PROG).elf $(PROG).c TinyWireS.a motor_controller.o
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avrdude -p t861 -P $(DEV) -c avrisp -b 19200 -U flash:w:$(PROG).hex

TinyWireS.a: TinyWireS.o USI_TWI_Slave.o
	avr-ar rcs TinyWireS.a TinyWireS.o USI_TWI_Slave.o

TinyWireS.o: TinyWireS/TinyWireS.cpp  
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c TinyWireS/TinyWireS.cpp 

USI_TWI_Slave.o: TinyWireS/utility/USI_TWI_Slave.c
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c TinyWireS/utility/USI_TWI_Slave.c

motor_controller.o: motor_controller.cpp
	avr-c++ -std=c++11 -O1 -mmcu=attiny861 -c motor_controller.cpp

clean:
	rm -rf *.o *.hex *.a

fuses:
	avrdude -p t861 -P $(DEV) -c avrisp -b 19200 -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m

reset:	# The v in the fuse parameters is for verify, might fail, but resets anyway.
	avrdude -p t861 -P $(DEV) -c avrisp -b 19200 -U lfuse:v:0xE1:m	