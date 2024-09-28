# MioLink

This project is a port of the [Black Magic Probe project](https://github.com/blackmagic-debug/blackmagic) to Raspberry Pi RP2040 (2 Cortex-M0+ cores at 125MHz) MCU.</br>
The project is a USB debugger that operates via JTAG and SWD interfaces, supporting many ARM Cortex processors ([see supported device list here](https://black-magic.org/supported-targets.html)).</br>
It also supports debug output via Serial Wire Output (SWO) and RTT. Additionally, the device has an extra UART serial port that can be accessed through a second Virtual COM port.</br>
This README contains only the basic information: information about the hardware, how to build the firmware and flash the probe.</br>
Detailed instructions on how to use the debugger, how to configure SWO, RTT and etc. will be posted later in [the project's wiki](https://github.com/Misaka0x2730/MioLink/wiki).

# Hardware
## General information
The main board for this device is MioLink: RP2040 + 16Mbit (2MiB) QSPI Flash memory (W25Q16).</br>
MioLink_Pico is a breakout board for Pico and Pico W featuring a power switch and voltage converters.</br>
Standard Pico and Pico W boards are also supported.</br>
The device type is determined at runtime, so all boards use the same firmware.</br>
All hardware CAD files (designed in KiCad) can be found [here](https://github.com/Misaka0x2730/MioLink/tree/main/hardware).</br>
[rev. A](https://github.com/Misaka0x2730/MioLink/tree/main/hardware/revA)/[rev. B](https://github.com/Misaka0x2730/MioLink/tree/main/hardware/revB) folders contain design files for MioLink rev. A and MioLink rev. B boards respectively.</br>
[MioLink_Pico](https://github.com/Misaka0x2730/MioLink/tree/main/hardware/MioLink_Pico) is an breakout board for Pico and Pico W.</br>
[MioLink_adapter](https://github.com/Misaka0x2730/MioLink/tree/main/hardware/MioLink_adapter) is an adapter that allows you to connect the debugger with target boards that have different types of connectors.</br>

## MioLink revisions comparsion
There are currently 2 hardware revisions of the MioLink board (rev.A and rev.B), with the following differences:

Power system: In rev.A, there is a hardware bug that causes the current value provided by the device on pin 1 (Vtref) of the debug connector to be unlimited,
as pin 5 of the TPS2553 is mistakenly connected to ground (pin 2) instead of the input voltage (pin 1).</br>
As a result, the current is limited only by the LDO (LP5907MFX-3.3), which means approximately 250mA of total current on the 3.3V line.

Rev.B uses a DC-DC converter to get 3.3V from 5V, and the current supplied to the debug connector is limited to around ~350 mA (min. 300, max. 400).

rev. A  |  rev. B
:------:|:-------:
![image](https://github.com/user-attachments/assets/ca20d195-47d2-4530-8daa-e9d0aed68b91) |  ![image](https://github.com/user-attachments/assets/b5c66c69-2552-469a-bd72-b8bd37ff3a03)

## Pinout
MioLink and MioLink_Pico use the standard ARM 10-pin 0.1" connector and has additional 4-pin 0.1" UART connector:</br>
![image](https://github.com/user-attachments/assets/98218707-b79a-4d30-ae67-68f5ac11d38c)


Pin | Descriptiuon | Pin | Description
:--:|:------------:|:---:|:-----------:
1 | VTref | 2 | SWDIO/TMS
3 | GND | 4 | SWDCLK/TCK
5 | GND | 6 | SWO/TDO/RX (Probe RX, Target TX)
7 | NC | 8 | NC/TDI/TX (Probe TX, Target RX)
9 | GND | 10 | RESET

UART connector pinout:
Pin | Descriptiuon | Pin | Description
:--:|:------------:|:---:|:-----------:
1 | RX (Probe RX, Target TX) | 2 | VTref
3 | TX (Probe TX, Target RX) | 4 | GND

## Connecting target to probe
### Powering target from probe
MioLink and MioLink_Pico are able to provide 3.3V power for target on VTref (pin 1).</br>
PLEASE, pay attention that Vtref and target's JTAG/SWD pins levels should match.
This means that Vtref pin MUST BE connected to external reference voltage (1.65V to 5.5V) for probe's level translators.</br>
it's optional in the case, when target is a 3.3V powered MCU, but you MUST power Vtref pin with 3.3V by using ```monitor tpwr enable``` GDB command.</br>
Pico and Pico W are also able to provide 3.3V power for target, but it is recommended to keep the load on this pin less than 300mA,</br>
for more information see chapter 2.1, page 8 in [Pico datasheet](https://datasheets.raspberrypi.com/pico/pico-datasheet.pdf) and chapter 2.1, page 9 in [Pico W datasheet](https://datasheets.raspberrypi.com/picow/pico-w-datasheet.pdf).

Board | Max current
:----:|:----------:
MioLink | rev.A: ~200 mA;</br>rev.B: ~350 mA (min. 300, max. 400)
MioLink_Pico | ~200 mA (min. 175, max. 234)
Pico or Pico W | ~300 mA from 3V3 pin,</br>see recommendations in</br>Pico and Pico W datasheets.

### JTAG
Pin | Description
:--:|:----------:
TMS | Test mode select
TCK | Test clock
TDO | Test data output
TDI | Test data input
RESET | Optional reset pin

### SWD
Pin | Description
:--:|:----------:
SWDIO | Data input/output
SWDCLK | SWD clock
RESET | Optional reset pin


# Building and flashing
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

Build steps:
1. Download and install [MSYS2](https://www.msys2.org/);
2. Run mingw64;
3. Upgrade packets, MinGW64 will be restarted:</br>```pacman -Syu```
4. Install all requirements:</br>```pacman -S git mingw-w64-x86_64-python mingw-w64-x86_64-make mingw-w64-x86_64-cmake mingw-w64-x86_64-arm-none-eabi-gcc mingw-w64-x86_64-gcc```
5. Clone this project with submodules:</br>```git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git```
6. Change current dir:</br>```cd MioLink/firmware```

Build debug image:
1. Create working directory:</br>```mkdir debug```
2. Go to working directory:</br>```cd debug```
3. Run cmake:</br>```cmake -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles" ..```
4. Build image:</br>```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

Build release image:
1. Create working directory:</br>```mkdir release```
2. Go to working directory:</br>```cd release```
3. Run cmake:</br>```cmake -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ..```
4. Build image:</br>```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory USB-MSC bootloader.

## How to flash the probe
1. Disconnect USB cable;
2. Press BOOT (BOOTSEL on Pico and Pico W) button;
3. Connect USB cable to PC;
4. Release BOOT (BOOTSEL on Pico and Pico W) button;
5. Drag-and-Drop ```MioLink.uf2``` file to mass storage device;
6. The device will be flashed and rebooted automatically, after which it is ready for use.
