TARGETMCU       ?= rx600

CROSS           := rx-elf-
CC              := $(CROSS)gcc
LDSCRIPT	:= rx62n.ld
FLASHER		:= rxusb -v -v -d
CFLAGS          := -Os -Wall -Werror -g -mcpu=$(TARGETMCU)
CFLAGS += -fdata-sections -ffunction-sections -Wl,--gc-sections

SRCS                    := main.c rxrf24.c rxrf24_util.c
PROG                    := main

all:                    $(PROG).elf

$(PROG).elf:    $(OBJS)
	$(CC) $(CFLAGS) -o $(PROG).elf $(SRCS) -T $(LDSCRIPT)

clean:
	-rm -f $(PROG).elf

install: $(PROG).elf
	$(FLASHER) $(PROG).elf
