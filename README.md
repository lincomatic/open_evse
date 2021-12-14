# OpenEVSE

Firmware for OpenEVSE controller used in OpenEVSE Charging Stations sold in the USA, and OpenEnergyMonitor EmonEVSE units sold in (UK/EU).

- OpenEVSE: <https://store.openevse.com/collections/all-products>
- EmonEVSE: <https://shop.openenergymonitor.com/evse/>

Based on OpenEVSE: Open Source Hardware J1772 Electric Vehicle Supply Equipment

## USA

TODO: add notes about USA OpenEVSE

## UK/EU

- Disable `AUTOSVCLEVEL` (autodetection is designed for split-phase)
- Charging level default to `L2`
- Set `MAX_CURRENT_CAPACITY_L2 32` (limit for single-phase charging in UK/EU)
- Add '.EU' to version number
- Enable LCD Redraw every couple of min (required for EMC/CE)

### EmonEVSE

EmonEVSE (non-tethered type-2 EVSE unit)

- `PP_AUTO_AMPACITY` enabled to set max current based on non-tethered cable connected
- Three-phase option with `THREEPHASE` enabled to calculate three-phase energy

## API Documentation

- WIFI API: <http://github.com/openevse/ESP32_WiFi_V4.x/>
- RAPI API: <https://github.com/openenergymonitor/open_evse/blob/master/firmware/open_evse/rapi_proc.h>

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
- Highly recommend using the tested pre-compiled firmware (see releases page)

## Flash pre-compiled using avrdude

`$ avrdude -p atmega328p -B6 -c usbasp -P usb -e -U flash:w:firmware.hex`

ISP programmer required e.g [USBASP](https://www.amazon.co.uk/Hobby-Components-USBASP-Programmer-Adapter/dp/B06XYV162N)

### Set AVR fuses

This only needs to be done once in the factory

`avrdude -c USBasp -p m328p -U lfuse:w:0xFF:m -U hfuse:w:0xDF:m -U efuse:w:0xFD:m -B6`

If writing eFuse fails ISBasp may need a [firmware update](https://www.vishnumaiea.in/articles/electronics/how-to-solve-usbasp-avr-efuse-write-problem-on-progisp)

***

Tip Jar: I developed/maintain this firmware on a volunteer basis. Any donation, no matter how small, is greatly appreciated.

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/lincomatic)

```text
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
