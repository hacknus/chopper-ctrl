# Chopper (STM32 DC Motor Controller with encoder)

Minimal STM32-based motor controller firmware written in Rust. Implements a PID loop, USB command interface and DFU
flash support via the existing cargo runner.

Hardware:

- STM32 Thing Plus (Sparkfun) STM32F405RGT6 MCU
- DC motor
- Motor driver (MOSFET, or H-Bridge)
- Rotary encoder (simple single channel)

Code could easily be extended for directional control (H-bridge) and quadrature encoders.

## Features

- PID motor control task
- USB command/housekeeping interface
- DFU-ready flashing via dfu-util (configured in `.cargo/config.toml`)

## Prerequisites

- rustup + cargo
- rust target: thumbv7em-none-eabihf
    - Install with: rustup target add thumbv7em-none-eabihf
- arm toolchain (for objcopy):
    - e.g. Debian/Ubuntu: `sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi dfu-util`
- dfu-util (for DFU flashing)

## Build

From the project root:

- Release build:

  ```shell
  cargo build --release
  ```

The project is configured to build for `thumbv7em-none-eabihf` (see `.cargo/config.toml`).

## Flashing

- If python is installed (and pyserial), the board will automatically be reset into DFU mode when the cargo runner is
  executed.
  If not, you will need to manually put the board into DFU mode:
    - Reset the board into DFU mode (by setting BOOT0 high and resetting).
- The repository includes a cargo `runner` that converts the ELF to a binary and calls dfu-util.
- Run:
    ```shell
      cargo run --release
    ```

This will produce `firmware.bin` and flash it via dfu-util as configured.

## Usage / USB command interface

The firmware exposes a simple ASCII-based USB command interface. Commands and housekeeping messages use these prefixes:

- Commands are sent as lines containing `[CMD]` and terminated with `\r\n`.
    - Set speed: `[CMD] setSpeed=1.5\r\n`
    - Set PID gains:
        - `[CMD] setKp=0.9\r\n`
        - `[CMD] setKi=0.001\r\n`
        - `[CMD] setKd=100.0\r\n`
    - Control:
        - `[CMD] start\r\n`
        - `[CMD] stop\r\n`
        - `[CMD] reset\r\n`
    - Housekeeping control:
        - `[CMD] disableHK\r\n`
        - `[CMD] enableHK\r\n`
        - `[CMD] setHKRate=10\r\n`  (sets housekeeping rate in Hz; limited by firmware)
    - General:
        - `Ping\r\n` gets acknowledged with `Pong`
        - `[CMD] enter bootloader\r\n` reboots the device into the bootloader for reflashing.

- Acknowledgements and housekeeping are printed back over USB, prefixed with `[ACK]` and `[HK]`.


