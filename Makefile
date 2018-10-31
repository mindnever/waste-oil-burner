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
FLASH=28672
RAM=2500
F_CPU = 16000000
TICK_MS = 10
DEFINES += -DF_CPU=$(F_CPU)

BOARD = PROMICRO

OBJECTS = main.o usb_descriptors.o hid.o vcp.o usb.o twi.o lcd.o rx.o led.o microrl/src/microrl.o

DFU_PROGRAMMER=sudo dfu-programmer

LUFA_PATH = ../lufa-build

LUFA_CFLAGS = -I$(LUFA_PATH) -DARCH=ARCH_AVR8 -DBOARD=BOARD_$(BOARD) -DF_USB=$(F_CPU)UL -DUSE_LUFA_CONFIG_HEADER -I$(LUFA_PATH)/Config/ -DTICK_MS=$(TICK_MS)
LUFA_LIBS = $(LUFA_PATH)/obj/*.o


COMPILE = avr-gcc -Wall -O2 -std=gnu99 -I. -mmcu=$(DEVICE) $(DEFINES) $(LUFA_CFLAGS) -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffreestanding
LINK = avr-gcc $(LUFA_LIBS)  -Wl,--gc-sections -Wl,--relax -mmcu=$(DEVICE) -Wl,-u,vfprintf -lprintf_flt -lm

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
	rm -f firmware.lst firmware.obj firmware.cof firmware.list firmware.map firmware.eep.hex firmware.bin $(OBJECTS) usbdrv/*.o firmware.s usbdrv/oddebug.s usbdrv/usbdrv.s

# file targets:
firmware.bin:	$(OBJECTS)
	$(LINK) -o firmware.bin $(OBJECTS)

firmware.hex:	firmware.bin
	rm -f firmware.hex firmware.eep.hex
	avr-objcopy -j .text -j .data -O ihex firmware.bin firmware.hex
	./checksize firmware.bin $(FLASH) $(RAM)

size:
	./checksize firmware.bin $(FLASH) $(RAM)

# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

avrdude: firmware.hex
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

avrdude-nodep:
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:firmware.hex

flash: firmware.hex
	$(DFU_PROGRAMMER) $(DEVICE) erase || true
	$(DFU_PROGRAMMER) $(DEVICE) flash firmware.hex

launch:
	$(DFU_PROGRAMMER) $(DEVICE) start || true

disasm:	firmware.bin
	avr-objdump -d firmware.bin

cpp:
	$(COMPILE) -E main.c
