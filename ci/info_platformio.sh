#!/bin/sh

TOOLCHAIN=~/.platformio/packages/toolchain-atmelavr
AVR_CORE_DIR=~/.platformio/packages/framework-arduinoavr
ELF=.pio/build/$PIO_ENV/firmware.elf
HEX=.pio/build/$PIO_ENV/firmware.hex

source $(dirname $0)/info_common.sh
