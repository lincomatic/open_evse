
Firmware for OpenEVSE controller used in OpenEnergyMonitor EmonEVSE units solde in (UK/EU) via the OpenEnergyMonitor store. In collaboration with OpenEVSE.

https://shop.openenergymonitor.com/evse/

Based on OpenEVSE: Open Source Hardware J1772 Electric Vehicle Supply Equipment

## Changes by OpenEnergyMonitor:

- Disable `AUTOSVCLEVEL` (autodetection is designed for split-phase)
- Charging level default to `L2`
- Set `MAX_CURRENT_CAPACITY_L2 32` (limit for single-phase charging in UK/EU)
- Add '.EU' to version number
- Enable LCD Redraw every couple of min (required for EMC/CE)

### EmonEVSE

EmonEVSE (non-tethered type-2 EVSE unit)

- `PP_AUTO_AMPACITY`enabled to set max current based on non-tethered cable connected
- Three-phase option with `THREEPHASE`enabled to calculate three-phase energy

## API Documentation 

WIFI API: http://github.com/openevse/ESP32_WiFi_V3.x/
RAPI API: https://github.com/openenergymonitor/open_evse/blob/master/firmware/open_evse/rapi_proc.h

## Resources

- [OpenEnergyMonitor OpenEVSE Setup Guide](https://guide.openenergymonitor.org/integrations/openevse)
- [OpenEnergyMonitor OpenEVSE Shop](https://shop.openenergymonitor.com/ev-charging/)

- [OpenEVSE Controller Datasheet](https://github.com/OpenEVSE/OpenEVSE_PLUS/blob/master/OpenEVSE_PLUS_v5/OpenEVSE_Plus_v5.pdf)
- [OpenEVSE Controller Hardware Repo](https://github.com/OpenEVSE/OpenEVSE_PLUS)
- [OpenEVSE Project Homepage](https://openevse.com)


***


Firmware compile & upload help: [firmware/open_evse/LoadingFirmware.md](firmware/open_evse/LoadingFirmware.md)

NOTES:
- Working versions of the required libraries are included with the firmware code. This avoids potential issues related to using the wrong versions of the libraries.
- Platform IO/Arduino 1.6.x or newer is STRONGLY RECOMMENDED for compiling OpenEVSE. Older versions of Arduino will build sketches which are too big to fit into OpenEVSE, unless features are disabled.
- Current official build tools version is Arduino AVR Boards by Arduino v1.6.15. You can install any version of the GUI, but make sure to use that version of the build tools, as it was used for UL certification. WITH LATEST CHANGES, v1.6.15 results in a sketch that's too big (>32768). MUST DISABLE FEATURES TO BUILD A COMPATIBLE SKETCH.. disabling RAPI_WF does the trick.
->  Currently testing v1.6.23, which builds a sketch that fits, even when all options enabled. FOR DEVELOPMENT ONLY UNTIL FURTHER TESTING IS DONE

## Falsh pre-compiled using avrdude:

`$ avrdude -p atmega328p -c usbasp -P usb -e -U flash:w:firmware.hex`

ISP programmer required

***


```
Open EVSE is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3, or (at your option)
any later version.

Open EVSE is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Open EVSE; see the file COPYING.  If not, write to the
Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.

* Open EVSE is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```
