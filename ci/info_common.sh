#!/bin/sh

ARDUINO_VERSION=$(grep version= $AVR_CORE_DIR/platform.txt | cut -f2 -d=)
GCC_VERSION=$($TOOLCHAIN/bin/avr-gcc --version | head -n 1)
HEX_SIZE=$(ls -l $HEX | awk '{ print $5 }')
BIN_STATS=$($TOOLCHAIN/bin/avr-size $ELF | tail -n 1)
TEXT_SIZE=$(awk '{ print $1 }' <<< "$BIN_STATS")
DATA_SIZE=$(awk '{ print $2 }' <<< "$BIN_STATS")
BSS_SIZE=$(awk '{ print $3 }' <<< "$BIN_STATS")
TOTAL_SIZE=$(awk '{ print $4 }' <<< "$BIN_STATS")

echo
echo "Arduino AVR Version: $ARDUINO_VERSION"
echo "GCC Version: $GCC_VERSION"
echo "firmware.hex Size: $HEX_SIZE"
echo "Text Size: $TEXT_SIZE"
echo "Data Size: $DATA_SIZE"
echo "BSS Size: $BSS_SIZE"
echo "Total Size: $TOTAL_SIZE"
echo

if [ $1 ]; then
  cat <<EOF >$1
{
  "name": "$NAME",
  "built_tool": "$BUILT_TOOL",
  "arduino_version": "$ARDUINO_VERSION",
  "gcc_version": "$GCC_VERSION",
  "hex_size": $HEX_SIZE,
  "text_size": $TEXT_SIZE,
  "data_size": $DATA_SIZE,
  "bss_size": $BSS_SIZE,
  "total_size": $TOTAL_SIZE
}
EOF
fi
