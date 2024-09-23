# MioLink

This project is a port of the [Black Magic Probe project](https://github.com/blackmagic-debug/blackmagic) to a custom platform (MioLink) based on the RP2040 (2 Cortex-M0+ cores at 125MHz) with 16Mbit (2MiB) of QSPI-Flash memory (W25Q16).
The project is a USB debugger that operates via JTAG and SWD interfaces, supporting many ARM Cortex processors ([see supported device list here](https://black-magic.org/supported-targets.html)).
It also supports debug output via Serial Wire Output (SWO) and RTT. Additionally, the device has an extra UART serial port that can be accessed through a second Virtual COM port

# Building
## Requirements
The followding tools are required to build this project:
- git
- make
- Python
- cmake
- gcc
- arm-none-eabi-gcc

### Windows
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
