################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/display/subdir.mk
-include Drivers/STM32L1xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/IKS01A3/subdir.mk
-include Drivers/BSP/Components/stts751/subdir.mk
-include Drivers/BSP/Components/lsm6dso/subdir.mk
-include Drivers/BSP/Components/lps22hh/subdir.mk
-include Drivers/BSP/Components/lis2mdl/subdir.mk
-include Drivers/BSP/Components/lis2dw12/subdir.mk
-include Drivers/BSP/Components/hts221/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := Projet_GyroPlane
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Projet_GyroPlane.elf \

MAP_FILES += \
Projet_GyroPlane.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Projet_GyroPlane.list \


# All Target
all: main-build

# Main-build Target
main-build: Projet_GyroPlane.elf secondary-outputs

# Tool invocations
Projet_GyroPlane.elf Projet_GyroPlane.map: $(OBJS) $(USER_OBJS) C:\Users\falla\STM32CubeIDE\workspace_1.15.1\Projet_GyroPlane\STM32L152RETX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "Projet_GyroPlane.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\falla\STM32CubeIDE\workspace_1.15.1\Projet_GyroPlane\STM32L152RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Projet_GyroPlane.map" -Wl,--gc-sections -static -u _printf_float --specs=nano.specs -mfloat-abi=soft -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Projet_GyroPlane.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Projet_GyroPlane.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) Projet_GyroPlane.elf Projet_GyroPlane.list Projet_GyroPlane.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
