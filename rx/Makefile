MCPU    = atmega8
F_CPU   = 4000000
CC      = avr-gcc
CFLAGS  = -Wall -Os -DF_CPU=$(F_CPU) -mmcu=$(MCPU)
OBJCOPY = avr-objcopy

OBJECTS = main.o uart.o

default: compile

compile: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o main.elf
	rm -f main.hex
	$(OBJCOPY) -j .text -j .data -O ihex main.elf main.hex
	rm -f main.elf

clean:
	rm -f *.o *.elf *.hex *.a

%.o: %.c
	$(CC) $(CFLAGS) -c $<

fuse:
	# set to 4MHz internal
	avrdude -p m8 -U lfuse:w:0xe3:m -U hfuse:w:0xd9:m

flash:
	avrdude -p m8 -U flash:w:main.hex
