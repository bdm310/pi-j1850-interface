PRG            = pi-j1850-interface
OBJ            = main.o spi.o j1850.o
MCU_TARGET     = atmega328p
OPTIMIZE       = -O3
DEFS           = 
LIBS           = 
# You should not have to change anything below here.
CC             = avr-gcc
# Override is only needed by avr-lib build system.
override CFLAGS        = -g -Wall -g $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump
all: $(PRG).elf lst text eeprom fuse
$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)
# dependency:
demo.o: demo.c iocompat.h
clean:
	rm -rf *.o $(PRG).elf *.eps *.png *.pdf *.bak 
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES)
lst:  $(PRG).lst
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@
# Rules for building the .text rom images
text: hex
#bin srec
hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec
%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@
%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@
# Rules for building the .eeprom rom images
eeprom: ehex
#ebin esrec
ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec
%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@ \
	|| { echo empty $@ not generated; exit 0; }
%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@ \
	|| { echo empty $@ not generated; exit 0; }
%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@ \
	|| { echo empty $@ not generated; exit 0; }
# Rules for building the .fuse rom images
fuse: hex bin
#bin srec
hex:  $(PRG)_fuse.hex
bin:  $(PRG)_fuse.bin
srec: $(PRG)_fuse.srec
%_fuse.hex: %.elf
	$(OBJCOPY) -j .fuse -j .data -O ihex $< $@
%_fuse.srec: %.elf
	$(OBJCOPY) -j .fuse -j .data -O srec $< $@
%_fuse.bin: %.elf
	$(OBJCOPY) -j .fuse -j .data -O binary $< $@
# Every thing below here is used by avr-libc's build system and can be ignored
# by the casual user.
FIG2DEV                 = fig2dev
EXTRA_CLEAN_FILES       = *.hex *.bin *.srec
dox: eps png pdf
eps: $(PRG).eps
png: $(PRG).png
pdf: $(PRG).pdf
%.eps: %.fig
	$(FIG2DEV) -L eps $< $@
%.pdf: %.fig
	$(FIG2DEV) -L pdf $< $@
%.png: %.fig
	$(FIG2DEV) -L png $< $@
flash:
	sudo systemctl stop spi.service
	sudo avrdude -c linuxspi -p m328p -P /dev/spidev0.0 -b 100000 -U flash:w:$(PRG).hex -U lfuse:w:0xD2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
#	sudo systemctl start spi.service
