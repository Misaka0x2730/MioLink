# CLAUDE.md

## Scope

These instructions are for firmware work in `firmware/`. MioLink also has hardware and documentation trees; do not apply firmware assumptions to `hardware/`, `docs/`, or `test_board/` unless the task explicitly crosses those boundaries.

## Project Summary

MioLink firmware is a Black Magic Probe port for RP2040-class boards. It builds a USB debug probe with SWD/JTAG, GDB CDC, target UART CDC, DFU runtime, SWO trace, RTT support, target power/VTref monitoring, and runtime board detection.

Core technologies:

- RP2040 / Pico SDK, CMake, C11, C++17, ASM.
- FreeRTOS SMP for RP2040 with optional single-core builds.
- TinyUSB composite device: 2 CDC interfaces, 1 vendor trace interface, DFU runtime.
- PIO + DMA for SWD/JTAG timing-sensitive transfers.
- SEGGER RTT/SystemView in debug builds.
- Vendored dependencies under `firmware/external/`: Pico SDK, FreeRTOS-Kernel, TinyUSB, SEGGER, Black Magic.

## Important Directories

- `firmware/CMakeLists.txt`: top-level firmware target `MioLink`, generated headers, linker scripts, dependency wiring.
- `firmware/cmake/options.cmake`: public build options (`PICO_BOARD`, FreeRTOS cores/tick, RTT/SystemView).
- `firmware/cmake/bmp_targets.cmake`: Black Magic target-family feature switches.
- `firmware/external/external.cmake`: vendored dependency import and Black Magic source selection.
- `firmware/boards/`: Pico SDK board headers and board include path.
- `firmware/boards/pinout/`: board GPIO maps for MioLink rev A/B, MioLink_Pico, Pico, Pico W.
- `firmware/config/FreeRTOS/FreeRTOSConfig.h`: scheduler, heap, task affinity, timers.
- `firmware/config/tinyusb/tusb_config.h`: TinyUSB class/FIFO configuration.
- `firmware/config/SEGGER/`: RTT/SystemView configuration.
- `firmware/src/main.c`: FreeRTOS startup and Black Magic GDB loop task.
- `firmware/src/bmp_platform/`: platform glue, board detection, VTref/power, timing, monitor commands.
- `firmware/src/bmp_tap/`: SWD/JTAG transport over PIO/DMA.
- `firmware/src/pio_programs/`: `.pio` programs; generated headers are build artifacts.
- `firmware/src/usb/`: USB descriptors, TinyUSB task, CDC callbacks.
- `firmware/src/target_serial/`: second CDC target UART bridge with IRQ/DMA paths.
- `firmware/src/bmp_rtt_swo/`: RTT/SWO capture and trace streaming.
- `firmware/src/bmp_general/`: local Black Magic replacements/adapters.

## Do Not Edit Casually

- Do not modify `firmware/external/` unless the task is explicitly about vendored code or submodule integration.
- Do not edit generated/build directories: `firmware/build/`, `firmware/cmake-build-*`, `.cache/`, `.idea/`, `.vscode/`.
- Do not edit generated PIO headers; edit `.pio` sources and let CMake regenerate.
- Do not change pinout macros, USB endpoint numbers, PIO state-machine allocation, DMA IRQ sharing, or FreeRTOS affinity without checking every dependent module.

## Build Commands

Run from the repository root:

```sh
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=auto
cmake --build firmware/build -j
```

Useful release build:

```sh
cmake -S firmware -B firmware/build-release -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=auto
cmake --build firmware/build-release -j
```

Board selection:

- `-DPICO_BOARD=auto`: default runtime detection for MioLink rev A/B, MioLink_Pico, Pico, and Pico W.
- `-DPICO_BOARD=miolink`: MioLink rev A/B only.
- `-DPICO_BOARD=miolink_pico`: MioLink_Pico only.
- Pico SDK boards such as `pico`, `pico_w`, and `weact_studio_rp2040_2mb` are also supported.

Other common options:

- `-DCONFIG_NUMBER_OF_CORES=1` or `2`; RP2040 supports at most two cores.
- `-DCONFIG_FREERTOS_TICK_RATE_HZ=<100..10000>`.
- `-DCONFIG_ENABLE_RTT=ON/OFF`.
- `-DCONFIG_ENABLE_SYSVIEW=ON/OFF`; meaningful in debug builds and only enabled for single-core builds.
- `-DBMP_TARGET_<FAMILY>=ON/OFF` from `firmware/cmake/bmp_targets.cmake` to reduce included Black Magic target families.

Successful builds produce `MioLink.uf2` in the selected build directory. Flashing requires BOOTSEL/USB mass-storage or another explicit user-approved flashing method; do not flash hardware unless asked.

## Verification Expectations

For firmware changes, build at least the configuration most affected by the patch. Prefer these checks when relevant:

- General firmware change: Debug `PICO_BOARD=auto`.
- Release/linker/size-sensitive change: Release `PICO_BOARD=auto`; review linker memory usage.
- Board detection or GPIO pinout change: build `auto` plus the specific board (`miolink`, `miolink_pico`, `pico`, or `pico_w`) touched.
- FreeRTOS/SysView/core-affinity change: build dual-core and single-core if the code path differs.
- USB descriptor/TinyUSB change: build and, when hardware is available, enumerate the device and check both CDC ports, DFU runtime, and trace vendor interface.
- PIO/SWD/JTAG timing change: build and state clearly if no hardware-level SWD/JTAG validation was possible.

If hardware validation is not possible, say exactly what was built and what remains untested on device.

## Style

- Follow `firmware/.clang-format`: LLVM base, Linux braces, 4-space indent, 120-column limit, `PointerAlignment: Right`, `SortIncludes: false`.
- Follow `firmware/.clang-tidy` where practical; it targets local `src` code and intentionally disables some embedded-unfriendly checks.
- Preserve existing license headers. Much of the code is GPL-derived from Black Magic; config/vendor files may use MIT or other upstream licenses.
- When creating new firmware `.h` or `.c` files, use the headers and section skeletons in `firmware/FILE_TEMPLATES.md`.
- Prefer existing local naming: lower_snake_case functions/variables, UPPER_CASE macros, `MIOLINK_*` header guards.
- Keep comments useful for hardware timing, concurrency, USB descriptors, and board-specific behavior. Avoid narrating obvious C statements.

## Embedded Constraints

- FreeRTOS heap is small (`configTOTAL_HEAP_SIZE` is 24 KiB). Avoid new dynamic allocation in hot paths, protocol loops, and ISRs.
- Stack sizes are intentionally tight. If increasing stack or heap use, justify it and consider `monitor rtos_tasksinfo` / `monitor rtos_heapinfo`.
- Use FreeRTOS `FromISR` APIs from interrupts and preserve `BaseType_t higher_priority_task_woken` / yield behavior.
- Respect task affinity:
  - USB TinyUSB task runs on core 0 when affinity is enabled.
  - Target UART and SWO trace tasks run on core 0.
  - GDB task runs on core 1 in dual-core builds.
  - Timer service task is pinned to core 0.
- Do not block the TinyUSB task or hold off interrupts around USB/CDC paths.
- Be careful with DMA channel and IRQ sharing:
  - `DMA_IRQ_0` is shared by target UART and SWO trace.
  - `DMA_IRQ_1` is used by VTref ADC monitoring.
  - SWD/JTAG PIO uses `pio0`; SWD uses SM0, JTAG uses SM0/SM1.
  - `TAP_PIO_DMA_BUF_SIZE` is 16; do not exceed it without auditing all TAP paths.
- Target power is safety-critical. `monitor tpwr enable` can power a target through VTref on supported boards; default is off. Keep VTref, fault, ADC, and power-enable semantics conservative.

## Board and Pinout Rules

- Board headers are Pico SDK board headers. `firmware/CMakeLists.txt` sets `PICO_BOARD_HEADER_DIRS` so local headers in `firmware/boards/` take precedence.
- Runtime auto-detection is controlled by `BOARD_AUTO`, hardware version strap pins, and Pico W detection via CYW43/ADC probing.
- `platform_get_target_pins()`, `platform_get_led_pins()`, and `platform_get_vtref_info()` are the central runtime pin maps.
- If adding or changing a board:
  - Add or update `firmware/boards/*.h` and `firmware/boards/pinout/*.h`.
  - Update `firmware/src/bmp_platform/platform_boards.c`.
  - Check SWD PIO program selection in `firmware/src/bmp_tap/swdptap.c`.
  - Check USB identification strings generated by `platform_make_board_ident()`.
  - Build `PICO_BOARD=auto` and at least one specific board target.

## USB Rules

- Keep USB interface count, endpoint constants, descriptor sizes, and TinyUSB config in sync across:
  - `firmware/src/bmp_platform/platform.h`
  - `firmware/src/usb/usb.c`
  - `firmware/src/usb/usb_cdc.h`
  - `firmware/config/tinyusb/tusb_config.h`
- CDC 0 is the GDB server. CDC 1 is target serial UART.
- The vendor interface is for SWO trace when `PLATFORM_HAS_TRACESWO` is enabled.
- DFU runtime reboot uses `tud_dfu_runtime_reboot_to_dfu_cb()` and RP2040 `reset_usb_boot()`.

## Black Magic Integration Rules

- Upstream Black Magic code lives in `firmware/external/blackmagic`.
- Local replacements/adapters live in `firmware/src/bmp_general`, `firmware/src/bmp_platform`, `firmware/src/bmp_tap`, and `firmware/src/bmp_rtt_swo`.
- `firmware/external/external.cmake` intentionally excludes upstream `adiv5_swd.c` and `swdptap_generic.c` and uses local RP2040/PIO implementations instead.
- Keep `GDB_PACKET_BUFFER_SIZE`, platform macros, and `BMP_TARGET_*` feature definitions consistent with Black Magic expectations.

## Working Process

- Start by checking `git status --short` and avoid overwriting unrelated user changes.
- Use `rg` for code search.
- Keep patches small and localized. Firmware changes often have hardware timing, USB enumeration, or RTOS side effects.
- When touching CMake, prefer existing interface-library patterns and cache options.
- When changing build options or defaults, update this file and `docs/README.md` if user-facing behavior changes.
- Do not update submodules, fetch dependencies, or regenerate large vendored files unless the task explicitly requires it.
