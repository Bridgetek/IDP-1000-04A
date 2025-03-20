# Overview

**ImageSlideShow** is a software package created by EVE Screen Designer (ESD).

# Folder introduction
```
📂 smart_home_control_hub_app
    ├───Data                           | Contains all the Bitmap files
    ├───Esd_Core                       | The application core files
    ├───FT_Esd_Framework               | The application framework files
    ├───FT_Esd_Widgets                 | The widget files
    ├───FT_Eve_Hal                     | Hardware Abstraction layer for interfacing the RP2040 platform with EVE control support
    ├───ImageSlide                     | The application specific source code
    ├───ThirdPartyLib                  | Third party library
    ├───CMakeLists.txt                 | Cmake file
    ├───pico_build.bat                 | Shell script run command
    ├───pico_sdk_import.cmake          | SDK import file from pico
```
# Usage
**IDP-1000-04A** is using EVE BT817 with Raspberry Pi Pico RP2040 host platform. Users are expected to be familiar with the BT81X programming guide and data sheet for EVE BT817 chip.

## Hardware requirement
* USB cable
* power supply
* SD card 




## Software requirement
* This folder does not include Pico toolchain. For information on downloading, installing, and using the toolchain, please visit https://github.com/raspberrypi/pico-setup-windows. This folder used toolchain version is **1.5.1**.
* cmake
* Visual Studio
* GNU Arm Embedded Toolchain for windows

## Build instruction
1. Execute the pico_build.bat file from the command line or double-click it to run directly.
2. ImageSlide.uf file will be generated in **build** folder

## Program
1. While holding the **BOOTSEL** button, connect the board to computer via the USB cable, or alternatively, press the **RESET** button while holding the **BOOTSEL** button, then release the **RESET** button.
2. Release the **BOOTSEL** button. the board will enter bootloader mode and appear as a removable storage device (RPI-RP2) on computer.
3. Simply drag and drop the generated .uf2 firmware file onto the RPI-RP2 storage device.
