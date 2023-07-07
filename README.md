# AIOLI-FOC (All In One Low I - FOC)

Small, low current (I) closed loop simpleFOC driver board, that can be assembled for &lt;$10 with mostly basic components at LCSC. Has USB C for power, serial communication (USB CDC), and DFU firmware updates. The CANBUS peripheral is exposed on both edges for "stitching" boards together using the Kyocera B2B connector used on the STLINK-V3-MINIE. SWD + UART via STLINK 14 pin connector.

The target for this project was to create an all-in-one simpleFOC board with high resolution magnetic encoder, better MCU than the previous STM32F103, CANBUS for multi board communication, and be assembled with top-side only components. The board is 4 layers (which at this size, does not add any cost over the 2-layers). Additionally, it has a compact size of 40x40mm. 

![Photo of PCB](/single.jpg)
![Photo of PCBs](/many.jpg)