## Overview

This is firmware that bridges WiFi and UART for the ESP32-C6.

## Requirement

- Arduino IDE
- Arduino core for the ESP32 family of SoCs

## Usage

Simply open it in the Arduino IDE and compile.
We expect it to be rebuilt in the most up-to-date environment possible.

Settings related to operation are stored in non-volatile memory using Preferences, so they do not require a file system.
All settings are configured via USB using the terminal.
If incorrect settings cause startup issues, it is advisable to specify Erase All Flash when transferring the firmware.

The following commands can be executed with a single key press from USB console:
- ‘!’  
Enter ESP32 bootloader wait mode.
- ‘i’  
Echo current status.
- ‘f’  
Write default settings to non-volatile memory.
- ‘s’  
Perform various initial settings sequentially.
  - hostname: Any hostname (for mDNS name resolution)
  - mode: 0=OFF, 1=AP, 2=STA
  - ssid: If AP, your own SSID; if STA, the SSID of an existing WiFi network
  - psk: Pre-Shared Key
  - ip: Specify my IP address; if blank, assign from DHCP
  - mask: Specify my IP mask; if blank, assign from DHCP
  - port: Port number for waiting for connections from external applications
  - serial protocol: 0=OFF, 1=PUSR, 2=LsrMstInsert
  - baudrate: Initial baudrate
  - serial config: Initial serial configration

Incidentally, the method for transmitting the LineCoding information inserted via WiFi is selected using the serial protocol. PUSR refers to PUSR's proprietary protocol, while LsrMstInsert refers to a stream activated by IOCTL_SERIAL_LSRMST_INSERT. You can choose one encoding method from these two types.

## Licence

[MIT](https://github.com/mukyokyo/ESP32C6-WiFi-Serial-Bridge/blob/main/LICENSE.txt)
