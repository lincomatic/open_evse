#!/bin/sh

ARDUINO_VERSION=$(grep version= $AVR_CORE_DIR/platform.txt | cut -f2 -d=)
GCC_VERSION=$($TOOLCHAIN/bin/avr-gcc --version | head -n 1)
HEX_SIZE=$(ls -l $HEX | awk '{ print $5 }')

echo
echo "Arduino AVR Version: $ARDUINO_VERSION"
echo "GCC Version: $GCC_VERSION"
echo "Binary Size: $HEX_SIZE"
echo
$TOOLCHAIN/bin/avr-size $ELF
