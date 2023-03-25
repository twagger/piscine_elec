# Variables
################################################################################
SRCS		= srcs/main.c \
			  srcs/uart/uart.c \
			  srcs/i2c/i2c.c \
			  srcs/i2c/expander.c \
			  srcs/i2c/7segments.c \
			  srcs/i2c/led.c \
			  srcs/adc/adc.c \
			  srcs/spi/spi.c \
			  srcs/switch/switch.c \
			  srcs/timer/timer.c \
			  srcs/rgb/rgb.c \
			  srcs/utils/utils.c
BIN_FILE	= srcs/program.bin
HEX_FILE	= srcs/program.hex
HEADERS		= includes/
PORT		= /dev/ttyUSB0
F_CPU		= 16000000
BAUD		= 115200
DEVICE		= atmega328p
PROGRAMMER	= arduino

BINS		= $(SRCS:.c=.bin)

# Bins
################################################################################
CC			= avr-gcc
HEX			= avr-objcopy
BURN		= avrdude
RM			= rm -rf
COPY		= cp

# Flags
################################################################################
CFLAGS		= -Wall -mmcu=$(DEVICE) -O2 -DF_CPU=${F_CPU}
HEXFLAGS	= -O ihex
BURNFLAGS	= -v -p $(DEVICE) -b $(BAUD) -c $(PROGRAMMER) -F 

# Rules
################################################################################
%.bin: 			%.c
				$(CC) $(CFLAGS) -c $< -o $@ -I$(HEADERS)

.PHONY:			all
all:			hex

.PHONY:			hex
hex:			$(BINS)
				$(CC) $(CFLAGS) -o $(BIN_FILE) $(BINS)
				$(HEX) $(HEXFLAGS) $(BIN_FILE) $(HEX_FILE)

.PHONY:			flash
flash:
				$(BURN) $(BURNFLAGS) -U flash:w:$(HEX_FILE):i -P $(PORT)

.PHONY:			clean
clean:			
				$(RM) $(BINS) $(BIN_FILE) $(HEX_FILE)

.PHONY:			re
re:				clean all