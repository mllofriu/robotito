PROG=i2cpid
DEV=/dev/ttyACM0
LINKFLAGS=
CFLAGS=-O2 -std=c++11 -mmcu=attiny84
AVRDUDEFLAGS=-p t84 -P $(DEV) -c avrisp -b 19200 

all: $(PROG).c USI_TWI_Slave.o motor_controller.o
	avr-c++ $(LINKFLAGS) $(CFLAGS) -o $(PROG).elf $(PROG).c USI_TWI_Slave.o motor_controller.o
	avr-objcopy -j .text -j .data -O ihex $(PROG).elf $(PROG).hex
	avr-size $(PROG).hex
	avrdude $(AVRDUDEFLAGS)  -U flash:w:$(PROG).hex

USI_TWI_Slave.o: TinyWireS/utility/USI_TWI_Slave.c
	avr-c++ $(CFLAGS) -c TinyWireS/utility/USI_TWI_Slave.c

motor_controller.o: motor_controller.cpp
	avr-c++ $(CFLAGS) -c motor_controller.cpp

clean:
	rm -rf *.o *.hex *.a

reset:	# The v in the fuse parameters is for verify, might fail, but resets anyway.
	avrdude $(AVRDUDEFLAGS)  -U lfuse:v:0xE1:m	

fuses:
	avrdude $(AVRDUDEFLAGS) -U lfuse:w:0xe2:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m