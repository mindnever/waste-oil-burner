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

#BOARD ?= PROMICRO

ifeq ($(BOARD),)
$(error BOARD must be set)
endif


BUILDDIR = build/$(BOARD)
FW = $(BUILDDIR)/firmware

TICK_MS = 10
OBJECTS = main.o twi.o lcd.o rx.o led.o zones.o flame.o adc.o eeconfig.o microrl/src/microrl.o

ifeq ($(BOARD),PROMICRO)

DEVICE=atmega32u4
FLASH=28672
RAM=2500
F_CPU = 16000000

OBJECTS += usb_descriptors.o hid.o vcp.o usb.o lcd_i2c.o
DFU_PROGRAMMER=sudo dfu-programmer

LUFA_PATH = ../lufa-build

LUFA_CFLAGS = -I$(LUFA_PATH) -DARCH=ARCH_AVR8 -DF_USB=$(F_CPU)UL -DUSE_LUFA_CONFIG_HEADER -I$(LUFA_PATH)/Config/
LUFA_LIBS = $(LUFA_PATH)/obj/*.o

DEFINES += -DUSE_USB_VCP -DUSE_USB_HID

else
ifeq ($(BOARD),PROMINI)

DEVICE=atmega328p
FLASH=28672
RAM=2048
F_CPU = 8000000
AVRDUDE = avrdude -v -c arduino -p $(DEVICE) -P/dev/tty.SLAB_USBtoUART -b57600

OBJECTS += usb_stubs.o lcd_gpio.o

else
ifeq ($(BOARD),UNO)

DEVICE=atmega328p
FLASH=28672
RAM=2048
F_CPU = 16000000
AVRDUDE = avrdude -v -c arduino -p $(DEVICE) -P/dev/cu.wchusbserialfa2330 -b115200

OBJECTS += usb_stubs.o

endif
endif
endif

DEFINES += -DF_CPU=$(F_CPU) -DBOARD=BOARD_$(BOARD) -DBOARD_PROMICRO=100 -DBOARD_PROMINI=101 -DTICK_MS=$(TICK_MS)

COMPILE = avr-gcc -Wall -O2 -std=gnu99 -I. -mmcu=$(DEVICE) $(DEFINES) $(LUFA_CFLAGS) -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -ffreestanding
LINK = avr-gcc $(LUFA_LIBS)  -Wl,--gc-sections -Wl,--relax -mmcu=$(DEVICE) -Wl,-u,vfprintf -lprintf_flt -lm

# symbolic targets:
all:	$(FW).hex

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
	rm -f $(FW).lst $(FW).obj $(FW).cof $(FW).list $(FW).map $(FW).eep.hex $(FW).bin $(FW).s $(addprefix $(BUILDDIR)/, $(OBJECTS))

# file targets:
$(FW).bin:	$(addprefix $(BUILDDIR)/, $(OBJECTS))
	$(LINK) -o $@ $(abspath $^)

$(FW).hex:	$(FW).bin
	rm -f $(FW).hex $(FW).eep.hex
	avr-objcopy -j .text -j .data -O ihex $(FW).bin $(FW).hex
	./checksize $(FW).bin $(FLASH) $(RAM)

size:
	./checksize $(FW).bin $(FLASH) $(RAM)

# do the checksize script as our last action to allow successful compilation
# on Windows with WinAVR where the Unix commands will fail.

avrdude: $(FW).hex
	$(AVRDUDE) -U flash:w:$(FW).hex

avrdude-nodep:
	avrdude -c usbasp -p atmega8 -U lfuse:w:0x9f:m -U hfuse:w:0xc9:m -U flash:w:$(FW).hex

flash: $(FW).hex
	$(DFU_PROGRAMMER) $(DEVICE) erase || true
	$(DFU_PROGRAMMER) $(DEVICE) flash $(FW).hex

launch:
	$(DFU_PROGRAMMER) $(DEVICE) reset

disasm:	$(FW).bin
	avr-objdump -d $(FW).bin

cpp:
	$(COMPILE) -E main.c

$(BUILDDIR)/%.o : %.c
	@mkdir -p $(dir $@)
	$(COMPILE) -c $(CFLAGS) $(CPPFLAGS) $< -o $@
