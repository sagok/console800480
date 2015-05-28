################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../FreeRTOS/croutine.c \
../FreeRTOS/event_groups.c \
../FreeRTOS/heap_2.c \
../FreeRTOS/list.c \
../FreeRTOS/queue.c \
../FreeRTOS/tasks.c \
../FreeRTOS/timers.c 

OBJS += \
./FreeRTOS/croutine.o \
./FreeRTOS/event_groups.o \
./FreeRTOS/heap_2.o \
./FreeRTOS/list.o \
./FreeRTOS/queue.o \
./FreeRTOS/tasks.o \
./FreeRTOS/timers.o 

C_DEPS += \
./FreeRTOS/croutine.d \
./FreeRTOS/event_groups.d \
./FreeRTOS/heap_2.d \
./FreeRTOS/list.d \
./FreeRTOS/queue.d \
./FreeRTOS/tasks.d \
./FreeRTOS/timers.d 


# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/%.o: ../FreeRTOS/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Sourcery Windows GCC C Compiler'
	arm-none-eabi-gcc -DSTM32F207 -DUSEH_USB_HS -DUSE_USB_FS -I"D:\workspace\console800480" -I"D:\workspace\console800480\tasks" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Class" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Class\MSC" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Class\MSC\Inc" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Class\MSC\Src" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Core" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Core\Inc" -I"D:\workspace\console800480\tasks\usb\STM32_USB_HOST_Library\Core\Src" -I"D:\workspace\console800480\FreeRTOS" -I"D:\workspace\console800480\FreeRTOS\include" -I"D:\workspace\console800480\FreeRTOS\portable" -I"D:\workspace\console800480\FreeRTOS\portable\GCC" -I"D:\workspace\console800480\FreeRTOS\portable\GCC\ARM_CM3" -I"D:\workspace\console800480\src" -I"D:\workspace\console800480\STM32F2xx_HAL_Driver" -I"D:\workspace\console800480\STM32F2xx_HAL_Driver\Inc" -I"D:\workspace\console800480\STM32F2xx_HAL_Driver\Src" -I"D:\workspace\console800480\system" -I"D:\workspace\console800480\tasks\display" -I"D:\workspace\console800480\tasks\usb" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\CDC" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\CDC\Inc" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\CDC\Src" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\MSC" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\MSC\Inc" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Class\MSC\Src" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Core" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Core\Inc" -I"D:\workspace\console800480\tasks\usb\STM32_USB_Device_Library\Core\Src" -I"D:\workspace\console800480\tasks\SDMMC" -O0 -Wall -Wa,-adhlns="$@.lst" -funsigned-bitfields -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -mcpu=cortex-m3 -mthumb -g3 -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


