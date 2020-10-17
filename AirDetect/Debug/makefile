################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Perpherial/Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DHT11_TEST.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DHT11_TEST.list \

OBJCOPY_HEX += \
DHT11_TEST.hex \

OBJCOPY_BIN += \
DHT11_TEST.bin \


# All Target
all: DHT11_TEST.elf secondary-outputs

# Tool invocations
DHT11_TEST.elf: $(OBJS) $(USER_OBJS) C:\Users\10094\STM32CubeIDE\workspace_1.4.0\DHT11_TEST\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "DHT11_TEST.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\10094\STM32CubeIDE\workspace_1.4.0\DHT11_TEST\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DHT11_TEST.map" -Wl,--gc-sections -static -L"C:\Users\10094\STM32CubeIDE\workspace_1.4.0\DHT11_TEST\Perpherial\Src" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DHT11_TEST.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DHT11_TEST.list"
	@echo 'Finished building: $@'
	@echo ' '

DHT11_TEST.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "DHT11_TEST.hex"
	@echo 'Finished building: $@'
	@echo ' '

DHT11_TEST.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "DHT11_TEST.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
