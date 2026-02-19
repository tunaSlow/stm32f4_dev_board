################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/indev/lv_indev.c \
../Display_Driver/lvgl/src/indev/lv_indev_gesture.c \
../Display_Driver/lvgl/src/indev/lv_indev_scroll.c 

OBJS += \
./Display_Driver/lvgl/src/indev/lv_indev.o \
./Display_Driver/lvgl/src/indev/lv_indev_gesture.o \
./Display_Driver/lvgl/src/indev/lv_indev_scroll.o 

C_DEPS += \
./Display_Driver/lvgl/src/indev/lv_indev.d \
./Display_Driver/lvgl/src/indev/lv_indev_gesture.d \
./Display_Driver/lvgl/src/indev/lv_indev_scroll.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/indev/%.o Display_Driver/lvgl/src/indev/%.su Display_Driver/lvgl/src/indev/%.cyclo: ../Display_Driver/lvgl/src/indev/%.c Display_Driver/lvgl/src/indev/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-indev

clean-Display_Driver-2f-lvgl-2f-src-2f-indev:
	-$(RM) ./Display_Driver/lvgl/src/indev/lv_indev.cyclo ./Display_Driver/lvgl/src/indev/lv_indev.d ./Display_Driver/lvgl/src/indev/lv_indev.o ./Display_Driver/lvgl/src/indev/lv_indev.su ./Display_Driver/lvgl/src/indev/lv_indev_gesture.cyclo ./Display_Driver/lvgl/src/indev/lv_indev_gesture.d ./Display_Driver/lvgl/src/indev/lv_indev_gesture.o ./Display_Driver/lvgl/src/indev/lv_indev_gesture.su ./Display_Driver/lvgl/src/indev/lv_indev_scroll.cyclo ./Display_Driver/lvgl/src/indev/lv_indev_scroll.d ./Display_Driver/lvgl/src/indev/lv_indev_scroll.o ./Display_Driver/lvgl/src/indev/lv_indev_scroll.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-indev

