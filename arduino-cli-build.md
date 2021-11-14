# OpenEVSE build via Arduino CLI

Setup

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr@1.6.15
cp arduino/1.6.15/boards.local.txt ~/.arduino15/packages/arduino/hardware/avr/1.6.15/
cp arduino/1.6.15/programmers.txt ~/.arduino15/packages/arduino/hardware/avr/1.6.15/
```

Build

```bash
arduino-cli compile -b arduino:avr:openevse firmware/open_evse
```

Upload

```bash
arduino-cli upload -b arduino:avr:openevse -P usbasp firmware/open_evse -v
```
