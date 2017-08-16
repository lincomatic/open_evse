
# OpenEVSE Firmware Loading

## Introduction 

Loading firmware to the OpenEVSE requires an ISP. There are many AVR/Arduino compatible programmers available from Adafruit, Sparkfun and the OpenEVSE Store. This wiki is based on the OpenEVSE AVR ISP found here: https://openevse-store.myshopify.com/collections/frontpage/products/openevse-programmer

## Window Driver

Platform IO and Arduino IDE need the libusbK driver, not the driver you may have installed as part of the [How to Load OpenEVSE Firmware (WinAVR)](https://openevse.dozuki.com/Guide/How+to+Load+OpenEVSE+Firmware+%28WinAVR%29/7) guide.

1. Plug in USBasp
2. Download Zadig from http://zadig.akeo.ie
3. Start zadig
4. Options > List all devices
5. Select USBasp from the drop down menu
6. Select libusbK(v3.0.7.0) driver
7. Click Install

## Firmware

The latest firmware source code is located in GIThub: https://github.com/lincomatic/open_evse

### Platform IO

#### 1. Install Platform IO

Either install the command line only [PlatformIO Core](http://docs.platformio.org/en/latest/installation.html) or the download the [PlatfomIO IDE](http://platformio.org/platformio-ide).

If you already have Atom or VS Code installed you can just install the [Atom package](https://atom.io/packages/platformio-ide) or [VSCode extension](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide) respectively.

#### 2. Compile

`$ pio run`

The first time platformIO is ran the AVR arduino tool chain and all the required libs will be installed if required.

#### 3. Upload

`$ pio run -t program`

### Arduino IDE

<div class="alert alert-success">
Note: These following instrictions have been extracted from Google Code and are not complete/properly edited
</div>

The Arduino IDE includes an code editor, serial monitor, and will compile and upload your code to the OpenEVSE.

Download and install the Arduino IDE from here: http://arduino.cc/en/Main/Software

In the Arduino Software you can simply select the Arduino UNO in the Tools - Boards menu or add a custom entry for OpenEVSE.

Optional - To add a custom board to Arduino IDE, copy the "boards.txt" to the folder ~\Arduino\hardware\openevse\avr.


```
#
open_evse.name=OpenEVSE

open_evse.bootloader.low_fuses=0xFF
open_evse.bootloader.high_fuses=0xDA
open_evse.bootloader.extended_fuses=0x05

open_evse.upload.using=avrispmkii
open_evse.upload.maximum_size=32768

open_evse.build.mcu=atmega328p
open_evse.build.f_cpu=16000000L
open_evse.core=arduino
open_evse.build.variant=standard
```

After you do that (and restart Arduino), you'll find a new entry in your Boards menu called "OpenEVSE". Select that when preparing to upload firmware.

See also: http://blog.lincomatic.com/?p=10

### Setting Fuse Bits

If you are burning virgin chips, you must set the fuse bits prior to programming. Fuses only need to be set once and will retain the settings unless reset. If you've modified boards.txt as shown above, then select "burn bootloader" from the tools menu to set the fuses as appropriate. This step won't actually load a bootloader. It will simply set the fuses.

See also: https://code.google.com/p/open-evse/wiki/AVRDUDE for how to accomplish this.

## Enabling features

To enable a feature delete the `//` in front of the #define tag. To disable a feature add `//`...

Example to enable the Advanced Power Supply features (`ADVPWR`) Change the line:

```
//#define ADVPWR
```

to

```
define ADVPWR
```

Note: Use Extreme caution with WATCHDOG, if delays were added that exceed the timer the EVSE board will reset in an endless loop. It may be very difficult or impossible to recover.
