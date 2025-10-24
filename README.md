# AIOLI-FOC (All In One Low I - FOC)

Small, low current (I) closed loop simpleFOC driver board, that can be assembled for &lt;$10 with mostly basic components at LCSC. Has USB C for power, serial communication (USB CDC), and DFU firmware updates. The CANBUS peripheral is exposed on both edges for "stitching" boards together using the Kyocera B2B connector used on the STLINK-V3-MINIE. SWD + UART via STLINK 14 pin connector.

The target for this project was to create an all-in-one simpleFOC board with high resolution magnetic encoder, better MCU than the previous STM32F103, CANBUS for multi board communication, and be assembled with top-side only components. The board is 4 layers (which at this size, does not add any cost over the 2-layers). Additionally, it has a compact size of 40x40mm.

![Photo of PCB](/single.jpg)
![Photo of PCBs](/many.jpg)

# Building

- Install [platformio core](https://docs.platformio.org/en/latest/core/installation/methods/index.html)
- Run `pio run --environment aioli-foc`

Alternately you can use the vscode integration

# Flashing

There are two options for flashing - DFU mode, and with stlink hardware

## DFU Mode

DFU mode can be done via the USB port, but due to the st32's default settings, pin PB8 will conflict with the CAN XCVR line, causing the st32 to boot to DFU mode every reset, losing the firmware and requiring recompilation. This is fine for testing but you'll need to use stlink to change that setting, and to program the st32 without being in DFU mode.

To program in DFU mode, set `upload_protocol` to `dfu` in `platformio.ini`.

You can then use `pio run --target upload --environment aioli-foc` or press `Upload` in vscode.

Note that `Upload and Monitor` may not work because `dfu-util` tends to think the upload failed even if it didn't.

## stlink Mode
- Acquire an [stlink v3 minie](https://www.st.com/en/development-tools/stlink-v3minie.html) or equvalent
- Connect the stlink to the 1.27mm 2x7 header on the board labeled `stm32g431`. This will require either soldering a header in (it is not in the JLBPCB BOM), or just jamming it in and holding it long enough to make contact.
- Once per board you will have to use `STM32CubeProgrammer` or `STM32 ST-LINK Utility` to set the Option Byte under User Configuration named `nSWBOOT0` to unchecked. This will prevent the st32 from going into DFU mode on every reset.
- Make sure to set `upload_protocol` back to `stlink` in `platformio.ini`
- Run `pio run --target upload --target monitor --environment aioli-foc` or click `Upload and Monitor` in vscode
