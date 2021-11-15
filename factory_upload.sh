avrdude -c USBasp -p m328p -U lfuse:w:0xFF:m -U hfuse:w:0xDF:m -U efuse:w:0xFD:m -B6
avrdude -p atmega328p -B6 -c usbasp -P usb -e -U flash:w:openevse.hex

