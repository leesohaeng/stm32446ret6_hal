################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

BUILD_ARTIFACT_NAME := STM32F103_BLUE_LCD
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F103_BLUE_LCD.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F103_BLUE_LCD.list \

OBJCOPY_BIN += \
STM32F103_BLUE_LCD.bin \


# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: STM32F103_BLUE_LCD.elf secondary-outputs

# Tool invocations
STM32F103_BLUE_LCD.elf: $(OBJS) $(USER_OBJS) ../STM32F103C8_FLASH.ld
	arm-none-eabi-gcc -o "STM32F103_BLUE_LCD.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"../STM32F103C8_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F103_BLUE_LCD.map" -Wl,--gc-sections -static  -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F103_BLUE_LCD.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F103_BLUE_LCD.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F103_BLUE_LCD.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F103_BLUE_LCD.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy.exe -O ihex "STM32F103_BLUE_LCD.elf" "STM32F103_BLUE_LCD.hex"
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY: post-build

-include ../makefile.targets
