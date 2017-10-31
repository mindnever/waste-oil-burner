# Name: Makefile
# Project: Remote Sensor
# Author: Christian Starkjohann, modified for LCD2USB by Till Harbaum
# Creation Date: 2005-03-20
# Tabsize: 4
# Copyright: (c) 2005 by OBJECTIVE DEVELOPMENT Software GmbH
# License: Proprietary, free under certain conditions. See Documentation.
# This Revision: $Id: Makefile.avrusb,v 1.1 2007/01/14 12:12:27 harbaum Exp $

# DEFINES += -DBWCT_COMPAT 
# DEFINES += -DDEBUG_LEVEL=1
DEVICE=atmega32u4
DEFINES += -DF_CPU=8000000
COMPILE = avr-gcc -Wall -O2 -I. -mmcu=$(DEVICE) $(DEFINES)

OBJECTS = main.o


# symbolic targets:
all:	firmware.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

flash-nodep:
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

# Fuse high byte:
# 0xc9 = 1 1 0 0   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   + --------- EESAVE (don't preserve EEPROM over chip erase)
#        | | | +-------------- CKOPT (full output swing)
#        | | +---------------- SPIEN (allow serial programming)
#        | +------------------ WDTON (WDT not always on)
#        +-------------------- RSTDISBL (reset pin is enabled)
# Fuse low byte:
# 0x9f = 1 0 0 1   1 1 1 1
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (external >8M crystal)
#        | |  +--------------- SUT 1..0 (crystal osc, BOD enabled)
#        | +------------------ BODEN (BrownOut Detector enabled)
#        +-------------------- BODLEVEL (2.7V)
fuse:
	$(UISP) --wr_fuse_h=0xc9 --wr_fuse_l=0x9f


clean:
	rm -f firmware.lst firmware.obj firmware.cof firmware.list firmware.map firmware.eep.hex firmware.bin *.o usbdrv/*.o firmware.s usbdrv/oddebug.s usbdrv/usbdrv.s

# file targets:
firmware.bin:	$(OBJECTS)
	$(COMPILE) -o firmware.bin $(OBJECTS)

firmware.hex:	firmware.bin
	rm -f firmware.hex firmware.eep.hex
	avr-objcopy -j .text -j .data -O ihex firmware.bin firmware.hex
	./checksize firmware.bin 8192 960
# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

avrdude: firmware.hex
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

avrdude-nodep:
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

flash: firmware.hex
	dfu-programmer $(DEVICE) erase || true
	dfu-programmer $(DEVICE) flash firmware.hex
	
disasm:	firmware.bin
	avr-objdump -d firmware.bin

cpp:
	$(COMPILE) -E main.c
