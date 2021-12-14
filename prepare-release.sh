#!/bin/bash

# Script to compile all released enviroments and copy build artifacts to project root for easy upload to github releases. 

pio run -e emonevse
pio run -e openevse
pio run -e emonevse-3ph

cp .pio/build/emonevse/firmware.hex emonevse.hex
cp .pio/build/openevse/firmware.hex openevse.hex
cp .pio/build/emonevse-3ph/firmware.hex emonevse-3ph.hex
ls *.hex

echo "Done" 


