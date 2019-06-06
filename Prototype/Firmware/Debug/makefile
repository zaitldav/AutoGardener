################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
AutoGardener_Prototype.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
AutoGardener_Prototype.list \


# All Target
all: AutoGardener_Prototype.elf secondary-outputs

# Tool invocations
AutoGardener_Prototype.elf: $(OBJS) $(USER_OBJS) /home/zaitldav/Personal/AutoGardener/Prototype/Firmware/STM32L031K6TX_FLASH.ld
	arm-none-eabi-gcc -o "AutoGardener_Prototype.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"/home/zaitldav/Personal/AutoGardener/Prototype/Firmware/STM32L031K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="AutoGardener_Prototype.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

AutoGardener_Prototype.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "AutoGardener_Prototype.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
