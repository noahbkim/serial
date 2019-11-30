# Target
DEVICE = atmega328p
CLOCK = 16000000
PROGRAMMER = -c arduino -b 115200 -P /dev/tty.usbmodem*
FUSES = -U hfuse:w:0xde:m -U lfuse:w:0xff:m -U efuse:w:0x05:m

# Build
AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -std=c99
SOURCE = ../src
BUILD = build

build:
	mkdir -p $(BUILD)

serial.o: build
	$(COMPILE) -c $(SOURCE)/serial.c -o $(BUILD)/serial.o

main.o: serial.o build
	$(COMPILE) -c main.c -o $(BUILD)/main.o

main.elf: main.o
	$(COMPILE) $(BUILD)/*.o -o $(BUILD)/main.elf

main.hex: main.elf
	rm -f $(BUILD)/main.hex
	avr-objcopy -j .text -j .data -O ihex $(BUILD)/main.elf $(BUILD)/main.hex
	avr-size --format=avr --mcu=$(DEVICE) $(BUILD)/main.elf

flash: main.hex
	$(AVRDUDE) -U flash:w:$(BUILD)/main.hex:i

clean:
	rm -rf $(BUILD)
