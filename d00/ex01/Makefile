# Variables
################################################################################
SRC			= main.c
BIN_FILE	= main.bin
HEX_FILE	= main.hex
PORT		= /dev/ttyUSB0
F_CPU		= 16000000
BAUD		= 115200
DEVICE		= atmega328p
PROGRAMMER	= arduino

# Bins
################################################################################
CC			= avr-gcc
HEX			= avr-objcopy
BURN		= avrdude
RM			= rm -rf
COPY		= cp

# Flags
################################################################################
CFLAGS		= -Wall -mmcu=$(DEVICE)
HEXFLAGS	= -O ihex
BURNFLAGS	= -v -p $(DEVICE) -b $(BAUD) -c $(PROGRAMMER) -F 

# Rules
################################################################################
.PHONY:			all
all:			hex

.PHONY:			hex
hex:			$(OBJS)
				$(CC) $(CFLAGS) -DF_CPU=${F_CPU} $(SRC) -o $(BIN_FILE)
				$(HEX) $(HEXFLAGS) $(BIN_FILE) $(HEX_FILE)

.PHONY:			flash
flash:
				$(BURN) $(BURNFLAGS) -U flash:w:$(HEX_FILE):i -P $(PORT)

.PHONY:			clean
clean:			
				$(RM) $(BIN_FILE) $(HEX_FILE)

.PHONY:			re
re:				clean all
