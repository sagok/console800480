################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := cs-rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include tasks/usb/STM32_USB_HOST_Library/Core/Src/subdir.mk
-include tasks/usb/STM32_USB_HOST_Library/Class/MSC/Src/subdir.mk
-include tasks/usb/STM32_USB_Device_Library/Core/Src/subdir.mk
-include tasks/usb/STM32_USB_Device_Library/Class/MSC/Src/subdir.mk
-include tasks/usb/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include tasks/usb/subdir.mk
-include tasks/display/ili9320/subdir.mk
-include tasks/display/Fonts/subdir.mk
-include tasks/display/subdir.mk
-include tasks/SDMMC/subdir.mk
-include system/subdir.mk
-include src/subdir.mk
-include STM32F2xx_HAL_Driver/Src/subdir.mk
-include FreeRTOS/portable/GCC/ARM_CM3/subdir.mk
-include FreeRTOS/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \

SECONDARY_LIST += \

SECONDARY_SIZE += \


# All Target
all: console800480.elf secondary-outputs

# Tool invocations
console800480.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Sourcery Windows GCC C Linker'
	arm-none-eabi-gcc -T"D:\projects\ARM\pult800480_Rev0101\system\STM32F217IG_FLASH.ld" -Xlinker --gc-sections -L"C:\Program Files (x86)\CodeSourcery\lib\gcc\arm-none-eabi\4.8.3\thumb2" -Wl,-Map,console800480.map -mcpu=cortex-m3 -mthumb -g3 -o "console800480.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

console800480.hex: console800480.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Flash Image'
	arm-none-eabi-objcopy -O ihex console800480.elf  "console800480.hex"
	@echo 'Finished building: $@'
	@echo ' '

console800480.lst: console800480.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Create Listing'
	arm-none-eabi-objdump -h -S console800480.elf > "console800480.lst"
	@echo 'Finished building: $@'
	@echo ' '

console800480.siz: console800480.elf
	@echo 'Invoking: ARM Sourcery Windows GNU Print Size'
	arm-none-eabi-size  --format=berkeley console800480.elf
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(SECONDARY_SIZE)$(OBJS)$(C_DEPS)$(ASM_DEPS)$(SECONDARY_FLASH)$(EXECUTABLES)$(SECONDARY_LIST)$(S_UPPER_DEPS) console800480.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
