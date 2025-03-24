# IDP-1000-04A Board Support Package

## Introduction
This repository offers hardware support for the IDP-1000-04A platform, excluding the EVE, LCD, and SD card, which are available at https://github.com/Bridgetek/IDP-1000-04A/tree/main/source/Default_firmware. While this BSP can function independently, without EVE support, the LCD will not be operational. A USB console is provided for basic functionality.

## Hardware Features
- RP2040 microcontroller (240kB SRAM, USB, etc) with 8 MByte MX25L6433F serial Flash
- EFM8bb1 microcontroller as GPIO expansion for RP2040
- Rotary ID switch interfaced to EFM8 micrcontroller
- KTD2061 LED controller for panel indicator lamps
- Micro USB port for RP2040
- Front panel submodule containing ambient light and TOF distance sensors

## Software Features
The BSP is designed as a support package for devices. Sample usage can be found in *bsp_test.c*

1. The application calls the BSP’s ```init_bsp()```function to initialize interfaces and peripherals:
    - RP2040 pico-sdk USB interface for debugging
    - RP2040 RTC realtime clock
    - RP2040 watchdog timer
    - RP2040 ADC and internal temperature sensor
    - I2C busses and peripherals
        - EFM8bb1 microcontroller, and obtaining rotary switch setting
        - Ambient light sensor (ALS)
        - Time-of-Flight (TOF) sensor
        - LED controller

2. The initialization results for the hardware peripherals listed above are stored in a single global C structure, ```BSPContexts_t dev_contexts```.

3. Periodically, the application calls the BSP's ```do_bsp_task()``` to service the interfaces and peripherals.

### Folder instroduction

```
📂 BSP
├── drivers          | collection of device drivers for IDP-1000-04A peripherals
│   ├── als          | LTR-308ALS I2C ambient light sensor
│   ├── efm8_mio     | I2C-interfaced EFM8 microcontroller used as GPIO expander
│   ├── i2c_utils    | Helpers for pico-sdk I2C device drivers
│   ├── led          | KTD2061 LED controller
│   └── tof          | VL53L3CX I2C time-of-flight distance sensor
├── example_binary   | A pre-compiled binary of this BSP for reference
├── extras           | CLI debug terminal sources
├── include          | configuration and build-time parameters
├── bsp_test.c       | main initialization and tasks routines
├── CMakeLists.txt   | CMake file
├── README.md        | this README file

```

### Depends
- This folder does not include Pico toolchain. For information on downloading, installing, and using the toolchain, please visit https://github.com/raspberrypi/pico-setup-windows. This folder used toolchain version is 1.5.1.
- GNU Arm Embedded Toolchain for windows
- cmake =>3.13
- Visual Studio 2019

### Build instruction
Because the IDP-1000-04A uses the Macronix QSPI-Flash, the pico-sdk built-in Pico module flash definition will not work. A replacement header file
```bridgetek_idp_1000_04a.h``` fixes this, but must be copied into the pico-sdk source tree.
```
$ cp include/bridgetek_idp_1000_04a.h <path to pico-sdk>/pico-sdk/src/boards/include/boards
```
 This will give visibility to the CMakelists.txt directive of
```
set(PICO_BOARD "bridgetek_idp_1000_04a")
```
Build:
```
set PICO_SDK_PATH=[path to pico-sdk]
set PICO_TOOLCHAIN_PATH=[path to GNU Arm Embedded Toolchain\10 2020-q4-major\bin]
mkdir build
cd build
[path to cmake] -G "NMake Makefiles" ..
nmake BSP_test
# The artifact BSP_test.uf2 built can be used to flash the RP2040.
```

### Program
1. While holding the **BOOTSEL** button, connect the board to computer via the USB cable, or alternatively, press the **RESET** button while holding the **BOOTSEL** button, then release the **RESET** button.
2. Release the **BOOTSEL** button. the board will enter bootloader mode and appear as a removable storage device (RPI-RP2) on computer.
3. Simply drag and drop the generated .uf2 firmware file onto the RPI-RP2 storage device.

### USB Debug-port Command Line Interface
The BSP supports a command-line interface when used with the USB-debug port and an appropriate Serial Terminal emulator on a host computer. The CLI supports the following commands:
- ```help```     - prints a list of commands
- ```cls```      - clear screen
- ```date```     - view or set datetime
- ```mr```       - pico flash content dump
- ```reboot```   - reboot the board
- ```i2cdetect```- display i2c bus information
- ```dev```      - display device list
- ```rot```      - display rotary value 
- ```led```      - LED test
- ```als```      - ALS test
