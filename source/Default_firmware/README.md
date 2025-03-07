# Overview
The "Default_firmware" folder includes reusable source code for applications, along with a sample application demonstrating a logo for reference purpose.

# Folder introduction
```
📂 Default_firmware
    ├───eve_hal                        | Hardware abstraction layer for interfacing the RP2040 platform with EVE BT817 control support
    ├───example_binary                 | A pre-compiled binary of this default firmware for reference
    ├───lib                            | Third party libraries
    │   ├──fatfs                       | fatfs library for SD card support
    ├───app.c                          | Sample application featuring RP2040 and BT817 initialization, calibration, and logo functionality
    ├───app.h                          | Header file for sample application
    ├───CMakeList.txt                  | cmake file
    ├───pico_sdk_import.cmake          | SDK import file from pico
```

# System diagram
Below diagram describe how this folder used in the program:
                     
                -------------------------------------------
                |             Main application            |
                -------------------------------------------
                     |                   |
                     |                   ▼
                     |        -----------------------
                     |        |          Lib        |
                     |        -----------------------
                     |                   |
                     ▼                   ▼
                -------------------------------------------
                |                  eve_hal                |
                -------------------------------------------
                                      |
                                      |
                                      ▼
                -------------------------------------------
                |             EVE ICs (BT81x)             |
                -------------------------------------------
                                      |
                                      |
                                      ▼   
                -------------------------------------------
                |               LCD, touch                | 
                -------------------------------------------

# Usage
IDP2040-101A is using EVE BT817 with Raspberry Pi Pico RP2040 host platform. Users are expected to be familiar with the BT81X programming guide and data sheet for EVE BT817 chip.

## Hardware requirement
* USB cable
* power supply
* IDP2040-101A development board

![IDP2040-101A](https://github.com/user-attachments/assets/87c11c64-84f7-4b9f-b9e7-034e60a9175f)

## Software requirement
* This folder does not include Pico toolchain. For information on downloading, installing, and using the toolchain, please visit https://github.com/raspberrypi/pico-setup-windows. This folder used toolchain version is 1.5.1.
* cmake
* Visual Studio
* GNU Arm Embedded Toolchain for windows

## Build instruction
1. Launch the Developer Command Prompt for VS in Visual Studio
```
set PICO_SDK_PATH=[path to pico-sdk]
set PICO_TOOLCHAIN_PATH=[path to GNU Arm Embedded Toolchain\10 2020-q4-major\bin]
cd Default_firmware
mkdir build
cd build
[path to cmake] -D PICO_DEFAULT_BOOT_STAGE2:STRING=boot2_generic_03h -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Debug ..
nmake IDP2040_101A
```
2. .uf file will be generated in __build__ folder

## Program
1. While holding the **BOOTSEL** button, connect the board to computer via the USB cable, or alternatively, press the **RESET** button while holding the **BOOTSEL** button, then release the **RESET** button.
2. Release the **BOOTSEL** button. the board will enter bootloader mode and appear as a removable storage device (RPI-RP2) on computer.
3. Simply drag and drop the generated .uf2 firmware file onto the RPI-RP2 storage device.
