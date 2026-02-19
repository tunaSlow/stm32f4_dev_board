################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.c 

OBJS += \
./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/sdl/%.o Display_Driver/lvgl/src/draw/sdl/%.su Display_Driver/lvgl/src/draw/sdl/%.cyclo: ../Display_Driver/lvgl/src/draw/sdl/%.c Display_Driver/lvgl/src/draw/sdl/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sdl

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sdl:
	-$(RM) ./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.cyclo ./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.d ./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.o ./Display_Driver/lvgl/src/draw/sdl/lv_draw_sdl.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sdl

