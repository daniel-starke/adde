adde
====

Arduino Development and Debug Environment.  
Used to debug issues of cross platform Arduino software projects in a standard PC environment using
the Arduino device as a proxy.

Features
========

- fully Arduino API conform
- encourages cross platform development
- full connection to the real Arduino periphery
- no need to re-flash the Arduino device during development to speed up compile/test cycles and preserve flash life-time
- rich debug and log functionality (user log entries via the `addeLog()` function)
- space is only limited by the development PC (not by the MCU)
- helps you find issues like out-of-bound, use-after-free, stack overflow, heap overflow, memory leaks and more
- <1ms remote command latency is possible
- enables the use of tools like GDB, Dr.Memory and Valgrind
- Linux and Windows are supported

Not included is support for MCU specific functions (like special registers or ASM commands) as well
as MCU or peripheral hardware emulation. Those features may be found in [simavr](https://github.com/buserror/simavr).

Usage
=====

Compile and flash the [adde firmware](src/arduino) with the Arduino IDE to your device.  
Copy your .ino and related files to the usr directory and execute `make clean all`.  
Run bin/adde with the serial port of your Arduino device.

Example
=======

Adde comes with a simple blink example in [usr](usr). Just compile everything with GCC using the provided make file

    make clean all

and run, e.g. on Windows, adde.exe (assuming Arduino is connected via COM1):

    bin/adde.exe COM1

Standard input and output will act as interface of Serial. That means if you read or write in Arduino from or to  
Serial it will be from or to the Windows console.  
Run `bin/adde.exe --help` for a list of options.

Building
========

The following dependencies are given:  
- C99
- C++11

Edit Makefile to match your target system configuration.  
Building the program:  

    make

Add one of the following flags to make if needed:
- NO_GPL=1
- UNICODE=1
- DEBUG=1
- USR=usr/

[![Linux GCC Build Status](https://img.shields.io/travis/daniel-starke/adde/master.svg?label=Linux)](https://travis-ci.org/daniel-starke/adde)
[![Windows Visual Studio Build Status](https://img.shields.io/appveyor/ci/danielstarke/adde/master.svg?label=Windows)](https://ci.appveyor.com/project/danielstarke/adde)    

Files
=====

|Name            |Meaning
|----------------|--------------------------------------------------
|**src/arduino/**|**Arduino device firmware for adde.**
|arduino.*       |Main implementation of the device functionality.
|Crc16.hpp       |CRC16 implementation according to [RFC 1662](https://tools.ietf.org/html/rfc1662).
|Framing.hpp     |Framing functions for the communication protocol.
|Get*            |Functions to provide Arduino specific constants.
|Meta.hpp        |C++11 meta programming helpers (derived from STL).
|New.cpp         |C++ placement new implementation.
|Optional.hpp    |Static allocated optional object creation helper.
|Pre.hpp         |C/C++ pre-processor helpers.
|Protocol.hpp    |Adde protocol specific constants.
|Reset.cpp       |Soft device reset implementation.
|Target.hpp      |Arduino target specific functions and macros.
|Utility.hpp     |Adde protocol utility functions.
|                |
|**src/utility/**|**Utility functions for the Arduino API.**
|adde.hpp        |Adde function return value helpers.
|argp*, getopt*  |Command-line parser.
|cvutf8.*        |UTF-8 conversion functions.
|mingw-unicode.h |Unicode enabled main() for MinGW targets.
|serial.*        |Serial interface functions.
|target.h        |Target specific functions and macros.
|tchar.*         |Functions to simplify ASCII/Unicode support.
|                |
|**src/**        |**Arduino API implementation.**
|abi.cpp         |C++ ABI specific functions.
|Arduino.*       |Arduino API implementation (most of it).
|binary.h        |Arduino binary constants.
|ConsoleSerial.h |Console interface for the Serial class.
|EEPROM.h        |Arduino EEPROM class interface.
|HardwareSerial.h|Arduino HardwareSerial class interface.
|License.i       |ADDE license text include.
|LiquidCrystal.h |Arduino LiquidCrystal class interface.
|new.*           |C++ `new` specific functions.
|pins_arduino.h  |Arduino PIN definitions.
|Print.*         |Arduino Print class and interface.
|Printable.h     |Arduino Printable class interface.
|SoftwareSerial.h|Arduino SoftwareSerial class interface.
|SPI.h           |Arduino SPI class interface.
|Stream.*        |Arduino Stream class and interface.
|WCharacter.*    |Arduino character specific functions.
|Wire.h          |Arduino Wire (I2C) class interface.
|WMath.cpp       |Arduino math functions.
|WString.*       |Arduino String class and interface.

Limitations
===========

- Callbacks are unimplemented (and so are `Wire.onReceive` and `Wire.onRequest`).
- `String::replace`, `String::remove`, `Stream::findUntil` and `Stream::findMulti` require GPL code and are excluded by default (see NO_GPL in [Makefile](Makefile))
- Remote functions returning pointers or modifying passed pointers or variables are not supported.
- The implemented soft reset executed at application startup does not resets all states on the remote device (see [`softReset()`](src/arduino/Reset.cpp).
- Primitive data types are handled with the same range as on the target device. However, implicit type promotion may create different results. This could be corrected with incorporation of `boost::safe_numerics` in future releases.

License
=======

See [copying file](doc/COPYING).  

Contributions
=============

No content contributions are accepted. Please file a bug report or feature request instead.  
This decision was made in consideration of the used license.
