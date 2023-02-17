# gd32f10x firmware library with gcc and makefile support

This is pre-converted gd32f10x firmware library with gcc and makefile support from GigaDevice official 'GD32F10x_Firmware_Library_V2.2.4.rar'.

For more information about how to use this library, refer to [this tutorial](https://github.com/cjacker/opensource-toolchain-stm32).

This firmware library support gd32f103/105/107 parts from GigaDevice:

The default part is set to 'gd32f103cbt6' for [WeAct GD32 Bluepill Plus](https://github.com/WeActStudio/WeActStudio.BluePill-Plus-GD32).

The default 'User' codes is blinking the LED connect to PB2.

To build the project, type `make`.


# to support other parts
To support other GD32F10x parts, you need:

- change 'Firmware/Ld/Link.ld', set FLASH and RAM size according to your MCU.
- choose correct startup asm file and change the 'ASM_SOURCES' in 'Makefile'
  + Firmware/CMSIS/GD/GD32F10x/Source/GCC/startup_gd32f10x_cl.S : for 105 and 107
  + Firmware/CMSIS/GD/GD32F10x/Source/GCC/startup_gd32f10x_hd.S : for flash size range from 256K to 512K
  + Firmware/CMSIS/GD/GD32F10x/Source/GCC/startup_gd32f10x_md.S : for flash size range from 16K to 128K 
  + Firmware/CMSIS/GD/GD32F10x/Source/GCC/startup_gd32f10x_xd.S : for flash size > 512K
- change `-DGD32F10X_MD` C_DEFS in 'Makefile' to `MD`, `HD`, `XD` or `CL` according to your MCU.
- change the 'TARGET' in 'Makefile'

