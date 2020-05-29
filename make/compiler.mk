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

# make sure that the GNU-ARM toolset exists...
ifeq ("$(wildcard $(GNU_ARM))","")
$(error GNU_ARM toolset not found. Please adjust the Makefile)
endif

AS     := $(GNU_ARM)/bin/$(ARM_EABI)-gcc
CC     := $(GNU_ARM)/bin/$(ARM_EABI)-gcc
CPP    := $(GNU_ARM)/bin/$(ARM_EABI)-gcc -E
LINK   := $(GNU_ARM)/bin/$(ARM_EABI)-gcc
OBJCPY := $(GNU_ARM)/bin/$(ARM_EABI)-objcopy
SIZE   := $(GNU_ARM)/bin/$(ARM_EABI)-size

MKDIR  := mkdir
RM     := rm -rf
ECHO   := echo
                  