# Introduction
This tool allows you to convert a flash image into a UF2 file and program the EVE-connected flash via the Pico's USB interface.

# prerequirement
The version of Python used is Python 3.
```
python -m pip install tqdm
```

# Usage
1. While holding the **BOOTSEL** button, connect the board to computer via the USB cable, or alternatively, press the **RESET** button while holding the **BOOTSEL** button, then release the **RESET** button.
2. Release the **BOOTSEL** button. the board will enter bootloader mode and appear as a removable storage device (RPI-RP2) on computer.
3. Copy the flash bin file (e.g., BT81X_Flash.bin which is assumed to already include the requested BLOB header) that needs to be programmed to the EVE-connected flash into this folder.
4. Open windows' **Command Prompt**
```
cd [path to this repo]
python uf2conv4eve.py --input [BT81X_Flash.bin] -o output-Flash.uf2
```
5. Simply drag and drop the generated *output-Flash.uf2* into the BTEVE-FLASH pop-up window and wait for the programming to complete.

> Example:

![image](https://github.com/user-attachments/assets/b0c65698-ff87-4500-85cb-2a3a1fc3b902)

![image](https://github.com/user-attachments/assets/452c394e-b7b5-4d47-bdef-1f6570487829)


