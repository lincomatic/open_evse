This is the development branch - latest bleeding edge code here
The version in this repo is usually functional
DO NOT use any of the other branches, including the stable branch.. they're just for my own testing, or kept for historical purposes

Tip Jar: I developed/maintain this firmware on a volunteer basis. Any donation, no matter how small, is greatly appreciated.

[![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://www.paypal.me/lincomatic)


Firmware for OpenEVSE: Open Source Hardware J1772 Electric Vehicle Supply Equipment

NOTES:
- Working versions of the required libraries are included with the firmware code. This avoids potential issues related to using the wrong versions of the libraries.
- Arduino 1.6.x or newer is STRONGLY RECOMMENDED for compiling OpenEVSE. Older versions of Arduino will build sketches which are too big to fit into OpenEVSE, unless features are disabled.
- Current official build tools version is Arduino AVR Boards by Arduino v1.6.15. You can install any version of the GUI, but make sure to use that version of the build tools, as it was used for UL certification. WITH LATEST CHANGES, v1.6.15 results in a sketch that's too big (>32768). MUST DISABLE FEATURES TO BUILD A COMPATIBLE SKETCH.. disabling RAPI_WF does the trick.
->  Currently testing v1.6.23, which builds a sketch that fits, even when all options enabled. FOR DEVELOPMENT ONLY UNTIL FURTHER TESTING IS DONE
------------

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