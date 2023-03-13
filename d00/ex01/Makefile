# Variables
################################################################################
SRCS		= main.c
BIN_FILE	= main.bin
HEX_FILE	= main.hex
USB_DEV		= /dev/ttyUSB0
F_CPU		= 2000000

# Progs
################################################################################
CC			= avr-gcc
HEX			= avr-objcopy
BURN		= avrdude
RM			= rm -rf
COPY		= cp

# Flags
################################################################################
CFLAGS		= -Wall -Wextra -Werror -c -mmcu=atmega328p
HEXFLAGS	= -O ihex
BURNFLAGS	= -p m328p -b 115200 -c arduino -F 

# Rules
################################################################################
.PHONY:			all
all:			hex

.PHONY:			hex
hex:			$(OBJS)
				$(CC) $(CFLAGS) -DF_CPU=${F_CPU} $(SRCS) -o $(BIN_FILE)
				$(HEX) $(HEXFLAGS) $(BIN_FILE) $(HEX_FILE)

.PHONY:			flash
flash:
				$(BURN) $(BURNFLAGS) -U flash:w:$(HEX_FILE):i -P $(USB_DEV)

.PHONY:			clean
clean:			
				$(RM) $(BIN_FILE) $(HEX_FILE)

.PHONY:			re
re:				clean all
