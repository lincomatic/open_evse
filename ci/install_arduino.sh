#!/bin/bash
set -o xtrace

ARDUINO_DIR=~/.arduino15/packages/arduino
AVR_CORE_VERSION=${AVR_CORE_VERSION:-'1.6.15'}
AVR_CORE_DIR=$ARDUINO_DIR/hardware/avr/$AVR_CORE_VERSION

curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sudo BINDIR=/usr/local/bin sh
arduino-cli core install arduino:avr@$AVR_CORE_VERSION
mkdir -p $AVR_CORE_DIR
cp arduino/1.6.15/boards.local.txt $AVR_CORE_DIR
