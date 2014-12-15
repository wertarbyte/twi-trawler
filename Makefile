MCU = attiny2313
F_CPU = 8000000
TARGET = trawler
SRC = trawler.c motor.c usiTwiSlave.c
COMBINE_SRC = 0

include avr-tmpl.mk
