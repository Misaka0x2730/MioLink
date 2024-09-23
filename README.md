# MioLink

This is basic hardware debugger board (1 non-isolated Target interface + UART) with [Blackmagic Probe](https://github.com/blackmagic-debug/blackmagic) firmware inside.
This board is based on RP2040 (2 Cortex-M0+ cores) with 16Mbit on-board QSPI Flash (W25Q16).


# Building
## Requirements
The followding tools are required to build this project:
- git
- make
- Python 3
- cmake
- gcc
- arm-none-eabi-gcc

### Windows
The simplest way to build the firmware under Windows environment is to use MinGW64.

Build steps:
1. Download and install [MSYS2](https://www.msys2.org/);
2. Run mingw64;
3. Upgrade packets, MinGW64 will be restarted:</br>```pacman -Syu```
4. Install all requirements:</br>```pacman -S git mingw-w64-x86_64-python mingw-w64-x86_64-make mingw-w64-x86_64-cmake mingw-w64-x86_64-arm-none-eabi-gcc  mingw-w64-x86_64-gcc```
5. Clone this project with submodules:</br>```git clone --recurse-submodules https://github.com/Misaka0x2730/MioLink.git```
6. Change current dir:</br>```cd MioLink/firmware```

Build debug image:
1. Create working directory:</br>```mkdir debug```
2. Go to working directory:</br>```cd debug```
3. Run cmake:</br>```cmake -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles" ..```
4. Build image:</br>```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory UF2 bootloader.

Build release image:
1. Create working directory:</br>```mkdir release```
2. Go to working directory:</br>```cd release```
3. Run cmake:</br>```cmake -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ..```
4. Build image:</br>```cmake --build .```
5. Use ```MioLink.uf2``` to flash device via factory UF2 bootloader.
