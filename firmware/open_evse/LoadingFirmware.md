
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

<div class="alert alert-success">
Note: Dev branch only
</div>

#### 1a. Install PlatformIO command line

The easiest way if running Linux is to install use the install script, this installed pio via python pip and installs pip if not present. See [PlatformIO installation docs](http://docs.platformio.org/en/latest/installation.html#installer-script). Or PlatformIO IDE can be used :

`$ sudo python -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"`

#### 1b. And / Or use PlatformIO IDE

Standalone built on GitHub Atom IDE, or use PlatformIO Atom IDE plug-in if you already have Atom installed. The IDE is nice, easy and self-explanitory.

[Download PlatfomIO IDE](http://platformio.org/platformio-ide)

#### 2. Hardware support file

Copy `arduino/openevse.json` to `~/.platformio/boards`

#### 3. Compile

`
$ pio run
`

The first time platformIO is ran the AVR arduino tool chain and all the required libs will be installed if required.

#### 4. Upload

`$ pio run -t upload`

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

### Loading Libraries

To finish setting up your Arduino IDE you will need to insure any required libraries, indicated by #include, are copied to the libraries folder of your Arduino installation.

OpenEVSE has two types of libraries those that will always be required by the core source code and those that are only required if you enable a certain feature.

The libraries below are not included in the basic Arduino installation.

The Time library is required by the core OpenEVSE firmware:

More info on the Time library can be found here: http://arduino.cc/playground/Code/Time

```

include

include

include

include

include

include

include // Required for RTC and Delay Timer

include

if defined(ARDUINO) && (ARDUINO >= 100)

include "Arduino.h"

else

include "WProgram.h" // shouldn't need this but arduino sometimes messes up and puts inside an #ifdef

endif // ARDUINO

```

Some features if enabled will also require a library, a good example are the RTC and LCDs options.

LCDs require LiquidTWI2 library: https://github.com/lincomatic/LiquidTWI2/downloads

RTC Requires RTClib and FlexiTimer2: https://github.com/adafruit/RTClib https://github.com/wimleers/flexitimer2

```

ifdef RGBLCD //Adafruit RGB LCD

include

include

define RED 0x1

define YELLOW 0x3

define GREEN 0x2

define BLUE 0x6

endif //Adafruit RGB LCD

ifdef I2CLCD

include

define LCD_I2C_ADDR 0 // for adafruit LCD backpack

endif // I2CLCD

```

The Adafruit RGB or i2c backpack LCD library LiquidTWI2.h can be found here:

https://github.com/lincomatic/LiquidTWI2/downloads

Enabling features

To enable a feature delete the "//" in front of the #define tag. To disable a feature add "//"...

Example to enable the Advanced Power Supply features (ADVPWR) Change the line:

//#define ADVPWR

to

define ADVPWR

Note: Use Extreme caution with WATCHDOG, if delays were added that exceed the timer the EVSE board will reset in an endless loop. It may be very difficult or impossible to recover.

```
//-- begin features

// enable watchdog timer //#define WATCHDOG

// GFI support

define GFI

// for stability testing - shorter timeout/higher retry count //#define GFI_TESTING

// serial port command line

define SERIALCLI

//Adafruit RGBLCD

define RGBLCD

// Adafruit LCD backpack in I2C mode //#define I2CLCD

// Advanced Powersupply... Ground check, stuck relay, L1/L2 detection. //#define ADVPWR

// single button menus (needs LCD enabled) 
// connect an SPST button between BTN_PIN and GND via a 2K resistor 
// How to use 1-button menu
// When not in menus, short press instantly stops the EVSE - another short press resumes. Long press activates menus
// When within menus, short press cycles menu items, long press selects and exits current submenu
//#define BTN_MENU

//-- end features 

```