################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.c 

OBJS += \
./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/opengles/%.o Display_Driver/lvgl/src/draw/opengles/%.su Display_Driver/lvgl/src/draw/opengles/%.cyclo: ../Display_Driver/lvgl/src/draw/opengles/%.c Display_Driver/lvgl/src/draw/opengles/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-opengles

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-opengles:
	-$(RM) ./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.cyclo ./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.d ./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.o ./Display_Driver/lvgl/src/draw/opengles/lv_draw_opengles.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-opengles

