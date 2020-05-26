# STM32H743ZI-Nucleo144 Examples

This is a collection of examples that run on the [STM32H743ZI-Nucleo144 devkit from STMicro](https://www.st.com/en/evaluation-tools/nucleo-h743zi.html)

## Getting Started

You'll need some way of flashing the nucleo board. You can use the built in ST-Link debugger. In my case, the devkit's ST-Link was flashed with Segger's Firmware that turns the ST-Link into a segger debugger.

Inside of each folder exists a Makefile. Running 

```
make clean; make -j all
```

will create a dbg folder with the appropriate binaries for flashing and/or debugging.

### Prerequisites

Development OS: All these examples were developed on Kubuntu 19.10 but Windows with msys2 with mingw64 should work fine. 

Compiler: gcc-arm-none-eabi-9-2019-q4-major but other compilers should work.

You'll have to modify the Makefile in each subdirectory to point to your installation of ARM GCC compiler. Look for a segment that looks like this:

```
#------------------------------------------------------------------------------
#  TOOLCHAIN SETUP - Common for all targets
#  GNU-ARM toolset (NOTE: You need to adjust to your machine)
#  see http://gnutoolchains.com/arm-eabi/
#------------------------------------------------------------------------------
ifeq ($(GNU_ARM),)

# For use with the sysinternals GCC for ARM cortex Mx series chips 
#GNU_ARM = /c/SysGCC/arm-eabi
#ARM_EABI = arm-eabi

# For use with the GNU with ARM extensions GCC for ARM cortex Mx series chips
#GNU_ARM = ~/workspace/gcc-arm-none-eabi-6-2017-q1-update
#ARM_EABI = arm-none-eabi

# For use with the GNU with ARM extensions GCC for ARM cortex Mx series chips
GNU_ARM = ~/workspace/gcc-arm-none-eabi-9-2019-q4-major
ARM_EABI = arm-none-eabi

endif
```

