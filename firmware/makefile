PRG            = pi-j1850-interface
OBJ            = main.o spi.o j1850.o
DEST		   = build
MCU_TARGET     = atmega328p
OPTIMIZE       = -O3
# You should not have to change anything below here.
CC             = avr-gcc
# Override is only needed by avr-lib build system.
override CFLAGS        = -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET)
override LDFLAGS       = -mmcu=$(MCU_TARGET) -Wl,-Map,$(DEST)/$(PRG).map
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
all: $(DEST)/$(PRG).elf lst text fuse

$(patsubst %.o,$(DEST)/%.o,$(OBJ)): $(patsubst %.o,%.c,$(OBJ))
	$(CC) $(CFLAGS) -o $@ -c $(patsubst build/%.o,%.c,$@)
	
$(DEST)/$(PRG).elf: $(patsubst %.o,$(DEST)/%.o,$(OBJ))
	$(CC) $(LDFLAGS) -o $@ $^
	avr-size -C --mcu=$(MCU_TARGET) $@
	
clean:
	rm -rf $(DEST)/*
lst:  $(DEST)/$(PRG).lst

$(DEST)/%.lst: $(DEST)/%.elf
	$(OBJDUMP) -h -S $< > $@
	
# Rules for building the .text rom images
text: hex
#bin srec
hex:  $(DEST)/$(PRG).hex
bin:  $(DEST)/$(PRG).bin
srec: $(DEST)/$(PRG).srec
$(DEST)/%.hex: $(DEST)/%.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
$(DEST)/%.srec: $(DEST)/%.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@
$(DEST)/%.bin: $(DEST)/%.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
# Rules for building the .eeprom rom images
eeprom: ehex
#ebin esrec
ehex:  $(DEST)/$(PRG)_eeprom.hex
ebin:  $(DEST)/$(PRG)_eeprom.bin
esrec: $(DEST)/$(PRG)_eeprom.srec
$(DEST)/%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }
$(DEST)/%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }
$(DEST)/%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }
# Rules for building the .fuse rom images
fuse: hex bin
#bin srec
hex:  $(DEST)/$(PRG)_fuse.hex
bin:  $(DEST)/$(PRG)_fuse.bin
srec: $(DEST)/$(PRG)_fuse.srec
$(DEST)/%_fuse.hex: $(DEST)/%.elf
	$(OBJCOPY) -j .fuse -j .data -O ihex $< $@
$(DEST)/%_fuse.srec: $(DEST)/%.elf
	$(OBJCOPY) -j .fuse -j .data -O srec $< $@
$(DEST)/%_fuse.bin: $(DEST)/%.elf
	$(OBJCOPY) -j .fuse -j .data -O binary $< $@
flash:
	sudo systemctl stop spi.service
	sudo avrdude -c linuxspi -p m328p -P /dev/spidev0.0 -b 100000 -U flash:w:$(DEST)/$(PRG).hex -U lfuse:w:0xD2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
#	sudo systemctl start spi.service
