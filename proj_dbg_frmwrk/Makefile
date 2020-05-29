##############################################################################
# Product: Makefile for STM32H743-Nucleo144 DevKit bring up (Bringup Demo)
#
# Copyright (C) 2020 Harry Rostovtsev. All rights reserved.
#
##############################################################################
# examples of invoking this Makefile:
# building configurations: Debug (default) and Release
#
# make CONF={rel|spy|dbg (default)} all
# make CONF={rel|spy|dbg (default)} clean
# 
# To control output from compiler/linker, use the following flag 
# If TRACE=0 -->TRACE_FLAG=
# If TRACE=1 -->TRACE_FLAG=@
# If TRACE=something -->TRACE_FLAG=something
TRACE                       = 0
TRACEON                     = $(TRACE:0=@)
TRACE_FLAG                  = $(TRACEON:1=)

# Get the current build date and time
NOW                        := $(shell date +%Y%m%d%H%M%S | tr ' :' '__')

# Output file basename (should be the same as the directory name)
TARGET                     := stm32h743-nucleo144
PROJECT_BASE               := proj_dbg_frmwrk
PROJECT_NAME               := $(PROJECT_BASE)-$(TARGET)
TARGET_HEX                  = $(BIN_DIR)/$(PROJECT_NAME).hex
TARGET_ELF                  = $(BIN_DIR)/$(PROJECT_NAME).elf
TARGET_BIN                  = $(BIN_DIR)/$(PROJECT_NAME).bin
TARGET_MAP                  = $(BIN_DIR)/$(PROJECT_NAME).map

#-----------------------------------------------------------------------------
# DIRECTORIES
#-----------------------------------------------------------------------------

# Top level directories
TOP_DIR                 = ../.

CMSIS_DIR               = $(TOP_DIR)/CMSIS
CMSIS_INC_DIR           = $(CMSIS_DIR)/Include

CMSIS_DEV_DIR           = $(CMSIS_DIR)/devices
CMSIS_DEV_INC_DIR       = $(CMSIS_DEV_DIR)/ST/STM32H7xx/Include

DEV_DRV_DIR             = $(TOP_DIR)/STM32
DEV_DRV_INC_DIR         = $(DEV_DRV_DIR)/inc
DEV_DRV_SRC_DIR         = $(DEV_DRV_DIR)/src
BRD_DRV_DIR             = $(DEV_DRV_DIR)/STM32H7xx_Nucleo_144
BRD_DRV_INC_DIR         = $(BRD_DRV_DIR)
BRD_DRV_SRC_DIR         = $(BRD_DRV_DIR)

PROJ_DIR                = $(TOP_DIR)/$(PROJECT_BASE)

# Get info about the compiler from our common mk file
-include $(TOP_DIR)/make/compiler.mk


# version.mk also must contain the version number of components
-include version.mk

#-----------------------------------------------------------------------------
# SOURCE VIRTUAL DIRECTORIES
#-----------------------------------------------------------------------------
VPATH                  += \
                          $(PROJ_DIR) \
                          $(TOP_DIR) \
                          $(DEV_DRV_SRC_DIR) \
                          $(BRD_DRV_SRC_DIR)                         
                         
#-----------------------------------------------------------------------------
# INCLUDE DIRECTORIES
#-----------------------------------------------------------------------------
INCLUDES               += \
                         -I$(PROJ_DIR) \
                         -I$(TOP_DIR) \
                         -I$(CMSIS_INC_DIR) \
                         -I$(CMSIS_DEV_INC_DIR) \
                         -I$(DEV_DRV_INC_DIR) \
                         -I$(BRD_DRV_INC_DIR)
                 
#-----------------------------------------------------------------------------
# BUILD OPTIONS FOR VARIOUS CONFIGURATIONS
#-----------------------------------------------------------------------------

# ARM CPU, ARCH, FPU, and Float-ABI types...
# ARM_CPU:   [cortex-m0 | cortex-m0plus | cortex-m1 | cortex-m3 | cortex-m4]
# ARM_ARCH:  [3| 4 | 6 | 7] (NOTE: must match ARM_CPU!)
# ARM_FPU:   [ | vfp]
# FLOAT_ABI: [ | soft | softfp | hard]
#
ARM_ARCH  := 7  
ARM_CPU   := -mcpu=cortex-m$(ARM_ARCH)

#ARM_FPU   := -mfpu=vfp
#FLOAT_ABI := -mfloat-abi=softfp
FLOAT_ABI := -mfloat-abi=soft


# add the pre-defined symbol for ARM architecture
#DEFINES += -D__ARM_ARCH=$(ARM_ARCH)
ASM_CPU += -Wa,--defsym,__ARM_ARCH=$(ARM_ARCH)

# add the pre-defined symbol __FPU_PRESENT if needed...
ifneq (,$(ARM_FPU))
DEFINES += -D__FPU_PRESENT
ASM_FPU += -Wa,--defsym,__FPU_PRESENT=1
endif

# Common options
MFLAGS                = $(ARM_CPU) $(ARM_FPU) $(FLOAT_ABI) \
                        -mthumb -std=gnu99 -mapcs -Wall \
                        -ffunction-sections -fdata-sections -fno-common \
						-ffreestanding -fno-builtin
						       
MFLAGS               += -Wno-unused-parameter \
						-fstack-usage
												
ASFLAGS               = -c $(ASM_CPU) $(ASM_FPU) $(MFLAGS)
CFLAGS                = -c $(MFLAGS)  \
						-W $(INCLUDES) $(DEFINES) $(DEP_OPTS)           

LINKFLAGS            += $(MFLAGS)  -Wl,--start-group -Wl,--end-group -Wl,-Map,$(TARGET_MAP) \
                         --specs=nano.specs -Xlinker -static  -Xlinker -z  \
                        -Xlinker muldefs  -Xlinker --defsym=__stack_size__=0x2000 \
                        -Xlinker --defsym=__heap_size__=0x0
                     
LINKFLAGS            += -Wl,--gc-sections
DEFINES              += -DUSE_FULL_LL_DRIVER -DSTM32H743xx

# Options dependent on build type
ifeq (rel, $(CONF))       # Release configuration ............................
    BIN_DIR          := rel
    DEFINES          += -DNDEBUG
    ASFLAGS          +=     
    CFLAGS           += -Os -Wno-unused-but-set-variable
    LINKFLAGS        += -Wl,--strip-all -Wl,--discard-all -Wl,--gc-sections
else ifeq (spy, $(CONF))  # Spy configuration ................................
    BIN_DIR          := spy
    DEFINES          += -DQ_SPY
    ASFLAGS          += -g3
    CFLAGS           += -Os -g3 -ggdb
    LINKFLAGS        += -g
    DEFINES          += -DMSG_DEBUG_ENABLE_EXT
else                     # default Debug configuration .......................
    BIN_DIR          := dbg
    DEFINES          += 
    ASFLAGS          += -g3   
    CFLAGS           += -O0 -g3 -ggdb 
    LINKFLAGS        += -g
endif
                     
# Linker script and path						
LD_SCRIPT            := $(TOP_DIR)/STM32H743ZITx_FLASH.ld -static

#-----------------------------------------------------------------------------
# DEFINES
#-----------------------------------------------------------------------------
# What processor we are compiling for
DEFINES              += -DSTM32H743xx

# Use Low Level (LL) drivers instead of this pile of excrement that is HAL
DEFINES              += -DUSE_FULL_LL_DRIVER

# Defines for processor and board specific functionality
DEFINES              += 
#-----------------------------------------------------------------------------
# FILES
#-----------------------------------------------------------------------------

# assembler source files
A_SRCS                 += startup_stm32h743xx.s

# C source files
C_SRCS                 += main.c \
                          stm32h7xx_it.c \
                          system_stm32h7xx.c \
                          stm32h7xx_hal_msp.c \
                          \
                          stm32h7xx_hal.c \
                          stm32h7xx_hal_cortex.c \
                          stm32h7xx_hal_dma.c \
                          stm32h7xx_hal_dma_ex.c \
                          stm32h7xx_hal_gpio.c \
                          stm32h7xx_hal_pwr.c \
                          stm32h7xx_hal_pwr_ex.c \
                          stm32h7xx_hal_rcc.c \
                          stm32h7xx_hal_rcc_ex.c \
                          stm32h7xx_hal_uart.c \
                          stm32h7xx_hal_uart_ex.c \
                          \
                          stm32h7xx_nucleo_144.c \
                          \
                          syscalls.c
                          
A_OBJS                 = $(patsubst %.s,%.o,$(A_SRCS))
A_OBJS_EXT             = $(addprefix $(BIN_DIR)/, $(A_OBJS))
A_DEPS_EXT             = $(patsubst %.o, %.d, $(A_OBJS_EXT))

C_OBJS                 = $(patsubst %.c,%.o,$(C_SRCS))
C_OBJS_EXT             = $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT             = $(patsubst %.o, %.d, $(C_OBJS_EXT))

# Make sure not to generate dependencies when doing cleans
NODEPS      := clean show showver ver clean_exe 
ifeq (0, $(words $(findstring $(MAKECMDGOALS), $(NODEPS))))
-include $(A_DEPS_EXT) $(C_DEPS_EXT)
endif

#-----------------------------------------------------------------------------
# BUILD TARGETS
#-----------------------------------------------------------------------------
# This sets a dependency between generating the proto files before compiling the
# rest of the source, allowing multicore compiles

.PHONY: all clean $(BIN_DIR) show clean_exe
.DEFAULT_GOAL := all

#DEP_OPTS = -MF $(BIN_DIR)/$(@F:.o=.d)

all: clean_exe $(TARGET_HEX) 

$(BIN_DIR):
	@echo --- Creating dir $@ 
	mkdir -p $@

$(TARGET_HEX): $(TARGET_ELF) | $(BIN_DIR)
	@echo --- Creating hex $@
	$(TRACE_FLAG)$(OBJCPY) -O binary $< $@
	$(TRACE_FLAG)mv $(TARGET_HEX) $(TARGET_BIN)
	@echo --- Creating bin $(TARGET_BIN)
	$(TRACE_FLAG)$(OBJCPY) -O ihex $< $@

$(TARGET_ELF): $(C_OBJS_EXT) $(A_OBJS_EXT) | $(BIN_DIR)
	@echo --- Building $(PROJECT_NAME)
	$(TRACE_FLAG)$(LINK) -T$(LD_SCRIPT) $(LINKFLAGS) $(LIB_PATHS) -o $@ $^ $(LIBS)
	$(SIZE) $(TARGET_ELF)
	
$(BIN_DIR)/%.o : %.c | $(BIN_DIR)
	@echo --- Compiling $(<F)
	$(TRACE_FLAG)$(CC) $(CFLAGS) -D BASE_FILE_NAME=\"$(<F)\" -MD -MP -c $< -o $@
	
$(BIN_DIR)/%.o : %.s | $(BIN_DIR)
	@echo --- Compiling $(<F)
	$(TRACE_FLAG)$(AS) $(ASFLAGS) $< -o $@
	
$(BIN_DIR)/%.o : %.S | $(BIN_DIR)
	@echo --- Compiling $(<F)
	$(TRACE_FLAG)$(AS) $(ASFLAGS) $< -o $@
	
#$(BIN_DIR)/%.d : %.c | $(BIN_DIR)
#	@echo --- Checking dependency for $(<F)
#	$(TRACE_FLAG)$(CC) -MM -MT $(@:.d=.o) $(CFLAGS) $< > $@
		
clean: 
	@echo --- Cleaning all binary files
	$(TRACE_FLAG)-$(RM) dbg rel spy

clean_exe:
	@echo --- Removing $(TARGET_HEX) $(TARGET_BIN) $(TARGET_ELF)
	$(TRACE_FLAG)-$(RM) $(TARGET_HEX) $(TARGET_BIN) $(TARGET_ELF)

show:
	@echo CONF             = $(CONF)
	@echo BIN_DIR          = $(BIN_DIR)
	@echo A_SRCS           = $(A_SRCS)
	@echo A_OBJS_EXT       = $(A_OBJS_EXT)
	@echo A_DEPS_EXT       = $(A_DEPS_EXT)
	@echo C_SRCS           = $(C_SRCS)
	@echo C_OBJS_EXT       = $(C_OBJS_EXT)
	@echo C_DEPS_EXT       = $(C_DEPS_EXT)
	@echo TARGET_HEX       = $(TARGET_HEX)
	@echo TARGET_BIN       = $(TARGET_BIN)
	@echo TARGET_ELF       = $(TARGET_ELF)
	@echo VPATH            = $(VPATH)
	@echo INCLUDES         = $(INCLUDES)
