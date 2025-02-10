# MioLink

This project is a port of the [Black Magic Probe project](https://github.com/blackmagic-debug/blackmagic) to Raspberry Pi RP2040 (2 Cortex-M0+ cores at 125MHz) MCU.  
The project is a USB debugger that supports a lot of ARM Cortex target processors ([see supported device list here](https://black-magic.org/supported-targets.html)) via SWD and JTAG interfaces.  
It also supports debug output via Serial Wire Output (SWO) and RTT. Additionally, the device has an extra UART serial port that can be accessed through a second Virtual COM port.  
This README contains only the basic information: information about the hardware, how to build the firmware and flash the probe.  
Detailed instructions on how to use the debugger, how to configure SWO, RTT and etc. you can find on [the project's wiki](https://github.com/Misaka0x2730/MioLink/wiki).

# Table of Content
- [Hardware](#hardware)
  - [General information](#general-information)
  - [Pinout](#pinout)
    - [MioLink and MioLink_Pico pinout](#miolink-and-miolink_pico-pinout)
      - [Target connector pinout](#target-connector-pinout)
      - [UART connector pinout](#uart-connector-pinout)
    - [Pico/Pico W pinout](#picopico-w-pinout)
  - [Connecting target to probe](#connecting-target-to-probe)
    - [Information about the Vtref pin (pin 1)](#information-about-the-vtref-pin-pin-1)
      - [Maximum output current of the Vtref pin (pin 1)](#maximum-output-current-of-the-vtref-pin-pin-1)
    - [JTAG pins description](#jtag-pins-description)
    - [SWD pins description](#swd-pins-description)
- [Building and flashing the probe](#building-and-flashing-the-probe)
  - [Requirements](#requirements)
  - [Build on Windows](#build-on-windows)
    - [Install all requirements and clone the repository](#install-all-requirements-and-clone-the-repository)
    - [Build debug image](#build-debug-image)
    - [Build release image](#build-release-image)
  - [Build on Ubuntu](#build-on-ubuntu)
    - [Install all requirements and clone the repository](#install-all-requirements-and-clone-the-repository-1)
    - [Build debug image](#build-debug-image-1)
    - [Build release image](#build-release-image-1)
  - [Build on Mac OS](#build-on-mac-os)
    - [Install all requirements and clone the repository](#install-all-requirements-and-clone-the-repository-2)
    - [Build debug image](#build-debug-image-2)
    - [Build release image](#build-release-image-2)
  - [How to flash the probe](#how-to-flash-the-probe)
- [Known issues](#known-issues)

# Hardware
## General information
The main board for this device is *MioLink* ([rev. A](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/revA/MioLink_revA.pdf)/[rev. B](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/revB/MioLink_revB.pdf)): RP2040 + 16Mbit (2MiB) QSPI Flash memory (W25Q16).  
At this moment, there are two revisions of *MioLink*: rev. A and rev. B.  
You can find information about their differences [here](https://github.com/Misaka0x2730/MioLink/wiki/MioLink-revisions-comparsion).  
[MioLink_Pico](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/MioLink_Pico/revA/MioLink_Pico_revA.pdf) is a breakout board for Pico and Pico W featuring a power switch and voltage converters.  
Standard Pico and Pico W boards are also supported.  
The device type is determined at runtime, so all boards use the same firmware.  
[MioLink_adapter](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/MioLink_adapter/revA/MioLink_adapter_revA.pdf) is an adapter that allows you to connect the probe with target boards that have different types of connectors.  
You can find all hardware CAD files (designed in KiCad) [here](https://github.com/Misaka0x2730/MioLink/tree/main/hardware).  

## Pinout
### MioLink and MioLink_Pico pinout
#### Target connector pinout
*MioLink* and *MioLink_Pico* use the standard ARM 10-pin 0.1" connector and has additional 4-pin 0.1" UART connector:  
![image](https://github.com/user-attachments/assets/98218707-b79a-4d30-ae67-68f5ac11d38c)

Pin | Name  | Description              | Pin | Name          | Description
:--:|:-----:|:------------------------:|:---:|:-------------:|:-----------------------------------------------------------------------:
1   | VTref | Target reference voltage | 2   | SWDIO/TMS     | SWD Data input/output / JTAG Test mode select
3   | GND   | Ground                   | 4   | SWDCLK/TCK    | SWD Clock / JTAG Test clock
5   | GND   | Ground                   | 6   | SWO/TDO/RX    | SWO Trace output / JTAG Test data output / UART Probe RX (Target TX)
7   | NC    | Not connected            | 8   | NC/TDI/TX     | Not connected for SWD / JTAG Test data input / UART Probe TX (Target RX)
9   | GND   | Ground                   | 10  | RESET         | Reset pin

#### UART connector pinout:

Pin | Name | Description                    | Pin | Name  | Description
:--:|:----:|:------------------------------:|:---:|:-----:|:------------------------:
1   | RX   | UART Probe RX (UART Target TX) | 2   | VTref | Target reference voltage
3   | TX   | UART Probe TX (UART Target RX) | 4   | GND   | Ground

### Pico/Pico W pinout
![image](https://github.com/user-attachments/assets/6acd1bba-45c8-459c-bf57-7183418560bd)

Pin       | Name       | Description
:--------:|:----------:|:------------------------------------------------------------------------:
10 (GP7)  | RESET      | Reset pin
11 (GP8)  | TX         | UART Probe TX (UART Target RX)
12 (GP9)  | RX         | UART Probe RX (UART Target TX)
13        | GND        | Ground
14 (GP10) | SWDCLK/TCK | SWD Clock / JTAG Test clock
15 (GP11) | SWDIO/TMS  | SWD Data input/output / JTAG Test mode select
16 (GP12) | NC/TDI/TX  | Not connected for SWD / JTAG Test data input / UART Probe TX (Target RX)
17 (GP13) | SWO/TDO/RX | SWO Trace output / JTAG Test data output / UART Probe RX (Target TX)
36        | 3V3        | 3.3V output from Pico

## Connecting target to probe
### Information about the Vtref pin (pin 1)
PLEASE, note that the Vtref pin (pin 1) on *MioLink* and *MioLink_Pico* MUST always be connected (internally or externally) to the logic level voltage of the target being debugged (from 1.65V to 5.5V) to power the level shifters inside the probe.  
If the target device operates at a 3.3V logic level, this pin can be powered internally by the probe, also in this case, the target can be powered by the probe if needed, and the maximum current is specified in the table below, but be careful, accidentally connecting anything to this pin that is not rated for 3.3V may lead to irreversible damage to the external device.  
3.3V power supply on this pin can be enabled with the GDB command ```monitor tpwr enable``` and disabled with the command ```monitor tpwr disable```. By default (after probe reset), it's disabled.  
Pico and Pico W are also able to provide 3.3V power for target through 3V3(OUT) pin (pin 36), but it is recommended to keep the load on this pin less than 300mA, for more information see chapter 2.1, page 8 in [Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf) and chapter 2.1, page 9 in [Pico W datasheet](https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf).  
In all other cases, the Vtref pin (pin 1) functions as an input and must be connected to the logic level voltage of the target device.  
The allowable voltage range is from 1.65V to 5.5V.  

#### Maximum output current of the Vtref pin (pin 1):

Board | Max current
:----:|:----------:
MioLink | rev.A: ~200 mA;  rev.B: ~350 mA (min. 300, max. 400)
MioLink_Pico | ~200 mA (min. 175, max. 234)
Pico or Pico W | ~300 mA from 3V3 pin,  see recommendations in  Pico and Pico W datasheets.

### JTAG pins description

Pin | Description
:----:|:----------:
Vtref | Target reference voltage
GND   | Ground
TMS   | Test mode select
TCK   | Test clock
TDO   | Test data output
TDI   | Test data input
RESET | Optional reset pin

### SWD pins description

Pin    | Description
:-----:|:----------:
Vtref  | Target reference voltage
GND    | Ground
SWDIO  | Data input/output
SWDCLK | Clock signal
RESET  | Optional reset pin

# Building and flashing the probe
## Requirements
The following tools are required to build this project:
- git
- make
- Python
- cmake
- gcc
- arm-none-eabi-gcc

## Build on Windows
The simplest way to build the firmware under Windows environment is to use MinGW64.

### Install all requirements and clone the repository
1. Download and install [MSYS2](https://www.msys2.org/);
2. Run mingw64;
3. Upgrade packets, MinGW64 will be restarted:  ```pacman -Syu```
4. Install all requirements:  ```pacman -S git mingw-w64-x86_64-python mingw-w64-x86_64-make mingw-w64-x86_64-cmake mingw-w64-x86_64-arm-none-eabi-gcc mingw-w64-x86_64-gcc```
5. Clone this project with submodules:  ```git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git```
6. Change current dir:  ```cd MioLink/firmware```

### Build debug image:
1. Create working directory:  ```mkdir debug```
2. Go to working directory:  ```cd debug```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

### Build release image:
1. Create working directory:  ```mkdir release```
2. Go to working directory:  ```cd release```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

## Build on Ubuntu
### Install all requirements and clone the repository
1. Download and install all requirements:
```
cd /opt
sudo wget "https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz"
sudo tar -xf arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz
sudo rm arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz
export PATH="$PATH":/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin
sudo apt-get install git python3 cmake
```
Note: ```export PATH``` will set the path environment variable only for the current session.  
If you want to set PATH permanently, you need to add ```export PATH="$PATH":/opt/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi/bin``` to your ```~/.profile``` or ```~/.bashrc```.

2. Clone this project with submodules into home directory:
```
cd ~
git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git
```
3. Change current dir:  ```cd MioLink/firmware```

### Build debug image:
1. Create working directory:  ```mkdir debug```
2. Go to working directory:  ```cd debug```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

### Build release image:
1. Create working directory:  ```mkdir release```
2. Go to working directory:  ```cd release```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

## Build on Mac OS
### Install all requirements and clone the repository
1. Open the terminal and run `xcode-select --install` to install Xcode Command Line Tools (includes `gcc` and `make`);
2. Download and install [CMake](https://cmake.org/download/) for your system;
3. Download and install [Arm GNU Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) for your system;
4. Download and install [Python 3](https://www.python.org/downloads/macos/);
5. Clone this project with submodules:
```
git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git
```
6. Change current dir:  ```cd MioLink/firmware```

### Build debug image:
1. Create working directory:  ```mkdir debug```
2. Go to working directory:  ```cd debug```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Debug -G "Unix Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

### Build release image:
1. Create working directory:  ```mkdir release```
2. Go to working directory:  ```cd release```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Release -G "Unix Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

## How to flash the probe
1. Disconnect USB cable;
2. Press BOOT (BOOTSEL on Pico and Pico W) button;
3. Connect USB cable to PC;
4. Release BOOT (BOOTSEL on Pico and Pico W) button;
5. Drag-and-Drop ```MioLink.uf2``` file to mass storage device;
6. The device will be flashed and rebooted automatically, after which it is ready for use.

# Known issues
1. [Issue with STM32F103 in the debug firmware](https://github.com/Misaka0x2730/MioLink/issues/2): When target is STM32F103, if MioLink is flashed with a debug firmware and interface frequency higher than 1MHz, firmware verification (compare-sections) almost always fails (one or more sections - MIS-MATCHED).  
This occurs when working over both SWD and JTAG, but it most often occurs on JTAG, much less frequently on SWD.  
Release firmware works fine, even at a frequency of 10MHz.  
It seems to be somehow related to context switching, as after moving the GDB task to a core 1, the issue on SWD started occurring much less frequently.  
Workaround: for debug firmware to work with STM32F103 (and probably with all STM32F1xx) use interface frequency 1MHz or lower (monitor frequency 1M).
Or just use the release firmware.
