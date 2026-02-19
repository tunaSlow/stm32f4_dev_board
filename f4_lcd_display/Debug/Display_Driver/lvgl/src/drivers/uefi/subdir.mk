################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.c \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.c \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.c \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.c \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.c \
../Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.c 

OBJS += \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.o \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.o \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.o \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.o \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.o \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.o 

C_DEPS += \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.d \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.d \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.d \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.d \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.d \
./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/drivers/uefi/%.o Display_Driver/lvgl/src/drivers/uefi/%.su Display_Driver/lvgl/src/drivers/uefi/%.cyclo: ../Display_Driver/lvgl/src/drivers/uefi/%.c Display_Driver/lvgl/src/drivers/uefi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-uefi

clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-uefi:
	-$(RM) ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_context.su ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_display.su ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_keyboard.su ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_pointer.su ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_indev_touch.su ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.cyclo ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.d ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.o ./Display_Driver/lvgl/src/drivers/uefi/lv_uefi_private.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-uefi

