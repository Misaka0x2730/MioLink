# MioLink Firmware Agent Rules

## Scope

These instructions apply to files under `firmware/`. MioLink also has hardware and documentation trees; do not apply firmware assumptions outside `firmware/` unless the task explicitly crosses those boundaries.

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
cmake -S firmware -B firmware/build -DCMAKE_BUILD_TYPE=Debug -DPICO_BOARD=auto_rp2040
cmake --build firmware/build -j
```

Useful release build:

```sh
cmake -S firmware -B firmware/build-release -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=auto_rp2040
cmake --build firmware/build-release -j
```

Board selection:

- `-DPICO_BOARD=auto_rp2040`: default runtime detection for MioLink rev A/B, MioLink_Pico, Pico, and Pico W.
- `-DPICO_BOARD=miolink`: MioLink rev A/B only.
- `-DPICO_BOARD=miolink_pico`: MioLink_Pico only.
- Pico SDK boards such as `pico`, `pico_w`, and `weact_studio_rp2040_2mb` are also supported.

Other common options:

- `-DCONFIG_NUMBER_OF_CORES=1` or `2`; RP2040 supports at most two cores.
- `-DCONFIG_FREERTOS_TICK_RATE_HZ=<100..10000>`.
- `-DCONFIG_ENABLE_SEGGER_RTT=ON/OFF` (firmware self-debug via Segger RTT; distinct from BMP target-RTT proxy, which is always compiled in).
- `-DCONFIG_ENABLE_SEGGER_SYSVIEW=ON/OFF`; meaningful in debug builds and only enabled for single-core builds.
- `-DBMP_TARGET_<FAMILY>=ON/OFF` from `firmware/cmake/bmp_targets.cmake` to reduce included Black Magic target families.

Successful builds produce `MioLink.uf2` in the selected build directory. Flashing requires BOOTSEL/USB mass-storage or another explicit user-approved flashing method; do not flash hardware unless asked.

## Verification Expectations

For firmware changes, build at least the configuration most affected by the patch. Prefer these checks when relevant:

- General firmware change: Debug `PICO_BOARD=auto_rp2040`.
- Release/linker/size-sensitive change: Release `PICO_BOARD=auto_rp2040`; review linker memory usage.
- Board detection or GPIO pinout change: build `auto_rp2040` plus the specific board (`miolink`, `miolink_pico`, `pico`, or `pico_w`) touched.
- FreeRTOS/SysView/core-affinity change: build dual-core and single-core if the code path differs.
- USB descriptor/TinyUSB change: build and, when hardware is available, enumerate the device and check both CDC ports, DFU runtime, and trace vendor interface.
- PIO/SWD/JTAG timing change: build and state clearly if no hardware-level SWD/JTAG validation was possible.

If hardware validation is not possible, say exactly what was built and what remains untested on device.

## Style

- Follow `firmware/.clang-format`: LLVM base, Linux braces, 120-column limit, `PointerAlignment: Right`, `SortIncludes: false`.
- Indent with **spaces only — never tabs**. One indent level is exactly 4 spaces (`IndentWidth: 4`, `UseTab: Never`). Continuation lines also use 4 spaces (`ContinuationIndentWidth: 4`).
- Hard line-length limit is 120 columns. This applies to code, comments (including Doxygen blocks), and string literals; wrap or split where needed instead of overflowing.
- Follow `firmware/.clang-tidy` where practical; it targets local `src` code and intentionally disables some embedded-unfriendly checks.
- Preserve existing license headers. Much of the code is GPL-derived from Black Magic; config/vendor files may use MIT or other upstream licenses.
- When creating new firmware `.h` or `.c` files, use the headers and section skeletons in `firmware/FILE_TEMPLATES.md`.
- Prefer existing local naming: lower_snake_case functions/variables, UPPER_CASE macros, `MIOLINK_*` header guards.
- Use `#if defined(MACRO)` / `#if !defined(MACRO)` instead of `#ifdef MACRO` / `#ifndef MACRO`. Exception: keep the conventional `#ifndef`/`#define` pattern for header include guards.
- Order `#include` directives so that system headers (angle-bracket form, e.g. `#include <stdlib.h>`, `#include <stdint.h>`) come **last** in the include list. Separate the trailing system-header block from the preceding includes (project headers, vendored headers, SDK headers) with exactly one blank line. Example:
  ```c
  #include "platform.h"
  #include "usb_cdc.h"
  #include "FreeRTOS.h"

  #include <stdint.h>
  #include <string.h>
  ```
  Not allowed:
  ```c
  #include <stdint.h>
  #include "platform.h"
  #include <string.h>
  #include "usb_cdc.h"
  ```
- Object-like `#define` constants must wrap their replacement value in parentheses. This applies to numeric literals, single-identifier aliases, and any compound expression. Exceptions: macros defined without a replacement value (pure feature flags such as `BOARD_AUTO_RP2040`), header include guards, function-like macros where each parameter is individually parenthesised in its use site, and **string-literal concatenations** that must remain usable as initialiser / `printf`-style arguments (wrapping the concatenation in parentheses would change semantics; example: `#define FIRMWARE_VERSION GIT_MIOLINK_VERSION ", BMP " GIT_BMP_VERSION`). Example:
  ```c
  #define MIOLINK_REVA_TARGET_TCK_PIN     (24)
  #define PICO_W_DETECT_CYW43_CS_PIN      (CYW43_DEFAULT_PIN_WL_CS)
  #define PICO_FLASH_SIZE_BYTES           (2 * 1024 * 1024)
  #define PICO_BOOT_STAGE2_CHOOSE_W25Q080 (1)
  ```
  Not allowed:
  ```c
  #define MIOLINK_REVA_TARGET_TCK_PIN     24
  #define PICO_W_DETECT_CYW43_CS_PIN      CYW43_DEFAULT_PIN_WL_CS
  #define PICO_FLASH_SIZE_BYTES           2 * 1024 * 1024
  #define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1
  ```
- In compound boolean expressions (combined with `&&`, `||`, or other logical/bitwise operators), every individual sub-expression must be wrapped in its own parentheses, even when C operator precedence would not require it. This applies to `if`, `while`, `for`, `do/while`, ternaries, `return` expressions, and assignments. Example:
  ```c
  if ((usb_get_config() != USB_CONFIG_STATE_CONFIGURED) ||
      (gdb_serial_get_dtr() == GDB_SERIAL_DTR_DEASSERTED)) {
      ...
  }
  ```
  Not allowed:
  ```c
  if (usb_get_config() != USB_CONFIG_STATE_CONFIGURED ||
      gdb_serial_get_dtr() == GDB_SERIAL_DTR_DEASSERTED) {
      ...
  }
  ```
- Always wrap the body of every control-flow statement in braces, even when the body is a single statement. This applies to `if`, `else`, `else if`, `while`, `for`, and `do/while`. Single-statement bodies without braces are not allowed. Example:
  ```c
  if (channel != 0U) {
      return len;
  }
  ```
  Not allowed:
  ```c
  if (channel != 0U)
      return len;
  ```
- Do not use `goto` (including `goto out` / `goto cleanup` idioms for shared error-path cleanup). If a function needs an unconditional cleanup step regardless of how its body exits, structure the code so the cleanup lives in an outer wrapper while the body sits in a helper that uses normal early `return`s; or repeat the cleanup at each exit point if a wrapper would add more friction than it removes. Example:
  ```c
  static bool do_work_inner(...)
  {
      if (failed_step_a()) {
          return false;
      }
      if (failed_step_b()) {
          return false;
      }
      return true;
  }

  static bool do_work(...)
  {
      acquire_resource();
      const bool ok = do_work_inner(...);
      release_resource();
      return ok;
  }
  ```
  Not allowed:
  ```c
  static bool do_work(...)
  {
      bool ok = false;
      acquire_resource();
      if (failed_step_a()) {
          goto out;
      }
      ok = true;
  out:
      release_resource();
      return ok;
  }
  ```
- Keep comments useful for hardware timing, concurrency, USB descriptors, and board-specific behavior. Avoid narrating obvious C statements.

## C Documentation And Initialization

- Do not leave variables uninitialized. Initialize at declaration using an explicit value, `{0}`, `NULL`, `false`, or a meaningful sentinel such as `-1` / `PIN_NOT_CONNECTED`.
- Add Doxygen comments for functions, types, global variables, file-scope variables, macros, and macro definitions. Local variables inside functions are exempt.
- For functions, place the Doxygen block next to the prototype/declaration:
  - Public functions: immediately before the prototype in the header.
  - Private `static` functions: immediately before the prototype in the `Private Functions Prototypes` section.
  - Do not duplicate the same Doxygen block above the implementation when a documented prototype already exists.
  - **Vendored-prototype exception:** when a function is declared in an unmodifiable upstream header (e.g. Black Magic `platform_support.h`, `gdb_if.h`, `swd.h`, `adiv5.h`) and we provide the implementation locally, the Doxygen block lives above the local implementation. Editing the vendored header to host the docs is not allowed (`firmware/external/` is read-only per the rules above).
- Any Doxygen comment that contains `\brief` must use the multi-line block form, even when the brief is the only tag. Single-line `/** \brief ... */` comments are not allowed.
  - Not allowed:
    ```c
    /** \brief Ring of DMA RX staging buffers receiving target UART bytes. */
    ```
  - Required:
    ```c
    /**
     * \brief Ring of DMA RX staging buffers receiving target UART bytes.
     */
    ```
  - Short trailing `/**< ... */` comments on the same line as a field, macro, or variable declaration are still allowed and do not need to be expanded.
- Doxygen blocks for functions and function-like macros must document every parameter and the return value (when the function returns a value). Do not omit `\param` for any parameter, and do not omit `\return` for non-`void` returns.
- Every `\param` entry must declare the parameter direction with `\param[in]`, `\param[out]`, or `\param[in,out]`. Bare `\param name` without a direction is not allowed.

## Comment Style

- Keep comments short and focused.
- Normal implementation comments should usually be 1-3 lines.
- Do not write long explanatory essays or multi-paragraph rationale blocks inside `.c` or `.h` files.
- Comments should explain only non-obvious hardware behavior, timing constraints, concurrency assumptions, protocol edge cases, safety-critical decisions, or upstream integration quirks.
- Do not describe what the code already says. A `\brief` that merely restates the function name or a comment that paraphrases the next statement is noise.
  - Not allowed:
    ```c
    /**
     * \brief Returns the current VTref voltage.
     */
    float platform_get_vtref_voltage(void);

    /* Increment the counter. */
    counter++;
    ```
  - Acceptable (adds information the code does not):
    ```c
    /**
     * \brief Returns the current VTref voltage in volts, sampled by the ADC IRQ on DMA_IRQ_1.
     */
    float platform_get_vtref_voltage(void);

    /* Counter wraps at 2^32; callers must compare with modular arithmetic. */
    counter++;
    ```
- Doxygen `\brief` text should be one concise sentence.
- Function Doxygen blocks should focus on purpose, side effects, and important constraints. Required `\param`/`\return` tags from `C Documentation And Initialization` still apply — this rule narrows the prose, not the tag coverage.
- For changed code, avoid adding large comment blocks unless the user explicitly asks for detailed inline documentation.

## Embedded Constraints

- FreeRTOS heap is small (`configTOTAL_HEAP_SIZE` is 27 KiB). Avoid new dynamic allocation in hot paths, protocol loops, and ISRs.
- Stack sizes are intentionally tight. If increasing stack or heap use, justify it and consider `monitor rtos_tasksinfo` / `monitor rtos_heapinfo`.
- Use FreeRTOS `FromISR` APIs from interrupts and preserve `BaseType_t higher_priority_task_woken` / yield behavior.
- Respect task affinity:
  - USB TinyUSB task runs on core 0 when affinity is enabled.
  - Target UART and SWO trace tasks run on core 0.
  - GDB task runs on core 1 in dual-core builds.
  - Timer service task is pinned to core 0.
  - CYW43 driver task (created by `pico_cyw43_arch_sys_freertos` on Wi-Fi boards) runs at priority 1 with a 256-word stack (`CYW43_TASK_PRIORITY` / `CYW43_TASK_STACK_SIZE` in `firmware/CMakeLists.txt`); no explicit core affinity is set, so the FreeRTOS-SMP scheduler may place it on either core.
- Do not block the TinyUSB task or hold off interrupts around USB/CDC paths.
- Be careful with DMA channel and IRQ sharing:
  - `DMA_IRQ_0` is shared by target UART and SWO trace.
  - `DMA_IRQ_1` is used by VTref ADC monitoring.
  - SWD/JTAG PIO uses `pio0`; SWD uses SM0, JTAG uses SM0/SM1.
  - `TAP_PIO_DMA_BUF_SIZE` is 16; do not exceed it without auditing all TAP paths.
- Target power is safety-critical. `monitor tpwr enable` can power a target through VTref on supported boards; default is off. Keep VTref, fault, ADC, and power-enable semantics conservative.

## Board and Pinout Rules

- Board headers are Pico SDK board headers. `firmware/CMakeLists.txt` sets `PICO_BOARD_HEADER_DIRS` so local headers in `firmware/boards/` take precedence.
- Runtime auto-detection is controlled by `BOARD_AUTO_RP2040`, hardware version strap pins, and Pico W detection via CYW43/ADC probing.
- `platform_get_target_pins()`, `platform_get_led_pins()`, and `platform_get_vtref_info()` are the central runtime pin maps.
- If adding or changing a board:
  - Add or update `firmware/boards/*.h` and `firmware/boards/pinout/*.h`.
  - Update `firmware/src/bmp_platform/platform_boards.c`.
  - Check SWD PIO program selection in `firmware/src/bmp_tap/swdptap.c`.
  - Check USB identification strings generated by `platform_make_board_ident()`.
  - Build `PICO_BOARD=auto_rp2040` and at least one specific board target.

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
- `firmware/external/external.cmake` intentionally excludes upstream `adiv5_swd.c`, `adiv5_jtag.c`, and `swdptap_generic.c` and uses local RP2040/PIO implementations instead.
- Keep `GDB_PACKET_BUFFER_SIZE`, platform macros, and `BMP_TARGET_*` feature definitions consistent with Black Magic expectations.

## When Unsure

- Prefer asking the user before changing hardware-facing behavior, build options, linker layout, USB descriptors, pin mappings, or vendored integration.
- If a requested change can be implemented in several ways, choose the smallest local change that preserves existing architecture.
- If validation cannot be completed locally, still make the code change, run available static/build checks, and clearly report the missing validation.

## Scope Discipline

- Do not fix unrelated warnings, formatting, Doxygen gaps, clang-tidy findings, or TODOs while working on a requested change. Mention them separately if they are relevant.
