ROOT := ../..
SDK_ROOT := $(ROOT)/third_party/ti_mspm0_sdk_2_05_01_00
BUILD_DIR := build

GCC_ARMCOMPILER ?=
ifneq ($(strip $(GCC_ARMCOMPILER)),)
CC := $(GCC_ARMCOMPILER)/bin/arm-none-eabi-gcc
SIZE := $(GCC_ARMCOMPILER)/bin/arm-none-eabi-size
else
CC := arm-none-eabi-gcc
SIZE := arm-none-eabi-size
endif

TARGET := $(BUILD_DIR)/car
SRC_DIRS := $(ROOT)/src/app $(ROOT)/src/control $(ROOT)/src/drivers $(ROOT)/src/tasks
SOURCES := $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c)) \
           $(ROOT)/platform/mspm0/ti_msp_dl_config.c
OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))
VPATH := $(SRC_DIRS) $(ROOT)/platform/mspm0

INCLUDES := -I$(ROOT)/src/app -I$(ROOT)/src/control \
            -I$(ROOT)/src/drivers -I$(ROOT)/src/tasks \
            -I$(ROOT)/platform/mspm0 \
            -I$(SDK_ROOT)/source \
            -I$(SDK_ROOT)/source/third_party/CMSIS/Core/Include

CFLAGS := @generated/device.opt $(INCLUDES) -std=c99 -Os -g3 \
          -mcpu=cortex-m0plus -march=armv6-m -mthumb -mfloat-abi=soft \
          -ffunction-sections -fdata-sections -Wall -Wextra \
          -Wno-unused-parameter -Wno-missing-braces

LDFLAGS := -nostartfiles -static -mcpu=cortex-m0plus -march=armv6-m \
           -mthumb -mfloat-abi=soft -Wl,--gc-sections \
           -Wl,-T,generated/device.lds.genlibs \
           -Wl,-T,generated/device_linker.lds \
           -Wl,-Map,$(TARGET).map -L$(SDK_ROOT)/source \
           -lgcc -lc -lm --specs=nano.specs --specs=nosys.specs

.PHONY: all clean size

all: $(TARGET).out size

$(BUILD_DIR):
	@mkdir $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).out: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

size: $(TARGET).out
	$(SIZE) $<

clean:
	-del /Q $(BUILD_DIR)\* 2>NUL
