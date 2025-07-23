# MSP430FR5994 Firmware for finding other devices
This firmware is designed to run on the MSP430FR5994 microcontroller and is used to find other devices in a network. It uses the UART interface for communication and can be flashed using the MSPFlasher tool.


- The UART baud rate is set to 9600. This can be changed in the `main.c` file.
## Things that need to be adjusted
- MSP_GNU_ROOT - Path to the MSP430-GCC toolchain. This can be changed in the `Makefile` file.
- Device - can be changed in the `Makefile` file. The default is `msp430fr5994`. 
- MSPFlasherPath - Path to the MSPFlasher tool. This can be changed in the `Makefile` file.


