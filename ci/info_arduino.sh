#!/bin/sh

ARDUINO_DIR=~/.arduino15/packages/arduino
AVR_CORE_VERSION=${AVR_CORE_VERSION:-'1.6.15'}
AVR_CORE_DIR=$ARDUINO_DIR/hardware/avr/$AVR_CORE_VERSION

# EVIL, should use `jq`
GCC_PACKAGE=$(grep "\"avr\"" ~/.arduino15/package_index.json -A 100 | grep "\"$AVR_CORE_VERSION\"" -A 100 | grep "\"avr-gcc\"" -A 1 | grep "\"version\"" | head -n 1 | cut -d \" -f4)
TOOLCHAIN=$ARDUINO_DIR/tools/avr-gcc/$GCC_PACKAGE

ELF=firmware/open_evse/open_evse.arduino.avr.openevse.elf
HEX=firmware/open_evse/open_evse.arduino.avr.openevse.hex

source $(dirname $0)/info_common.sh
