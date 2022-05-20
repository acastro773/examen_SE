#TOOLCHAIN=~/toolchain/gcc-arm-none-eabi-4_9-2014q4/bin
#PREFIX=$(TOOLCHAIN)/arm-none-eabi-
PREFIX=arm-none-eabi-

CMSIS=./CMSIS
DRIVERS=./drivers
UTILS=./utilities
BOARD=./BOARD

ARCHFLAGS=-mthumb -mcpu=cortex-m0plus
COMMONFLAGS=-g3 -Og -Wall -Werror $(ARCHFLAGS)

CFLAGS=-I$(CMSIS) -I$(DRIVERS) -I$(UTILS) -I$(BOARD) -DCPU_MKL46Z256VLL4 $(COMMONFLAGS)
LDFLAGS=$(COMMONFLAGS) --specs=nano.specs -Wl,--gc-sections,-Map,$(TARGET).map,-Tlink.ld
LDLIBS=

CC=$(PREFIX)gcc
LD=$(PREFIX)gcc
OBJCOPY=$(PREFIX)objcopy
SIZE=$(PREFIX)size
RM=rm -f

TARGET=main

SRC=$(wildcard *.c)
OBJ=$(patsubst %.c, %.o, $(SRC))

CMSISSRC=$(wildcard $(CMSIS)/*.c)
CMSISOBJ=$(patsubst %.c, %.o, $(CMSISSRC))

DRIVERSSRC=$(wildcard $(DRIVERS)/*.c)
DRIVERSOBJ=$(patsubst %.c, %.o, $(DRIVERSSRC))

UTILSSRC=$(wildcard $(UTILS)/*.c)
UTILSOBJ=$(patsubst %.c, %.o, $(UTILSSRC))

BOARDSRC=$(wildcard $(BOARD)/*.c)
BOARDOBJ=$(patsubst %.c, %.o, $(BOARDSRC))

all: build size
build: elf srec bin
elf: $(TARGET).elf
srec: $(TARGET).srec
bin: $(TARGET).bin

clean:
	$(RM) $(TARGET).srec $(TARGET).elf $(TARGET).bin $(TARGET).map $(OBJ)\
	$(CMSISOBJ) $(DRIVERSOBJ) $(UTILSOBJ) $(BOARDOBJ)

$(TARGET).elf: $(OBJ) $(CMSISOBJ) $(DRIVERSOBJ) $(UTILSOBJ) $(BOARDOBJ)
	$(LD) $(LDFLAGS) $^ $(LDLIBS) -o $@

%.srec: %.elf
	$(OBJCOPY) -O srec $< $@

%.bin: %.elf
	    $(OBJCOPY) -O binary $< $@

size:
	$(SIZE) $(TARGET).elf

flash: all
	openocd -f openocd.cfg -c "program $(TARGET).elf verify reset exit"
