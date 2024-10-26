# MioLink

This project is a port of the [Black Magic Probe project](https://github.com/blackmagic-debug/blackmagic) to Raspberry Pi RP2040 (2 Cortex-M0+ cores at 125MHz) MCU.  
The project is a USB debugger that operates via JTAG and SWD interfaces, supporting many ARM Cortex processors ([see supported device list here](https://black-magic.org/supported-targets.html)).  
It also supports debug output via Serial Wire Output (SWO) and RTT. Additionally, the device has an extra UART serial port that can be accessed through a second Virtual COM port.  
This README contains only the basic information: information about the hardware, how to build the firmware and flash the probe.  
Detailed instructions on how to use the debugger, how to configure SWO, RTT and etc. will be posted later in [the project's wiki](https://github.com/Misaka0x2730/MioLink/wiki).

# Hardware
## General information
The main board for this device is *MioLink* ([rev. A](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/revA/MioLink_revA.pdf)/[rev. B](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/revB/MioLink_revB.pdf)): RP2040 + 16Mbit (2MiB) QSPI Flash memory (W25Q16).  
At this moment, there are two revisions of *MioLink*, rev. A and rev. B. You can find information about their differences [here](https://github.com/Misaka0x2730/MioLink/wiki/MioLink-revisions-comparsion).
[MioLink_Pico](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/MioLink_Pico/revA/MioLink_Pico_revA.pdf) is a breakout board for Pico and Pico W featuring a power switch and voltage converters.  
Standard Pico and Pico W boards are also supported.  
The device type is determined at runtime, so all boards use the same firmware.  
[MioLink_adapter](https://github.com/Misaka0x2730/MioLink/blob/main/hardware/MioLink_adapter/revA/MioLink_adapter_revA.pdf) is an adapter that allows you to connect the probe with target boards that have different types of connectors.  
All hardware CAD files (designed in KiCad) can be found [here](https://github.com/Misaka0x2730/MioLink/tree/main/hardware).  

## Pinout
### MioLink and MioLink_Pico target connector pinout
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
Pico and Pico W are also able to provide 3.3V power for target, but it is recommended to keep the load on this pin less than 300mA, for more information see chapter 2.1, page 8 in [Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf) and chapter 2.1, page 9 in [Pico W datasheet](https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf).  
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

## Install all requirements on Windows
The simplest way to build the firmware under Windows environment is to use MinGW64.

Build steps:
1. Download and install [MSYS2](https://www.msys2.org/);
2. Run mingw64;
3. Upgrade packets, MinGW64 will be restarted:  ```pacman -Syu```
4. Install all requirements:  ```pacman -S git mingw-w64-x86_64-python mingw-w64-x86_64-make mingw-w64-x86_64-cmake mingw-w64-x86_64-arm-none-eabi-gcc mingw-w64-x86_64-gcc```
5. Clone this project with submodules:  ```git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git```
6. Change current dir:  ```cd MioLink/firmware```

## Install all requirements on Ubuntu
1. Download and install all requirements:
```
cd /opt
sudo wget "https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz"
sudo tar -xf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
sudo rm arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
echo 'export PATH="$PATH":/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin' >> ~/.bashrc
sudo apt-get install git python3 cmake
```
2. Clone this project with submodules:
```
cd ~
git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git
```
3. Change current dir:  ```cd MioLink/firmware```

Build debug image:
1. Create working directory:  ```mkdir debug```
2. Go to working directory:  ```cd debug```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

Build release image:
1. Create working directory:  ```mkdir release```
2. Go to working directory:  ```cd release```
3. Run cmake:  ```cmake -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ..```
4. Build image:  ```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

## How to flash the probe
1. Disconnect USB cable;
2. Press BOOT (BOOTSEL on Pico and Pico W) button;
3. Connect USB cable to PC;
4. Release BOOT (BOOTSEL on Pico and Pico W) button;
5. Drag-and-Drop ```MioLink.uf2``` file to mass storage device;
6. The device will be flashed and rebooted automatically, after which it is ready for use.
