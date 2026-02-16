################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.c \
../Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.c 

OBJS += \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.o \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.d \
./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/sw/blend/%.o Display_Driver/lvgl/src/draw/sw/blend/%.su Display_Driver/lvgl/src/draw/sw/blend/%.cyclo: ../Display_Driver/lvgl/src/draw/sw/blend/%.c Display_Driver/lvgl/src/draw/sw/blend/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw-2f-blend

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw-2f-blend:
	-$(RM) ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_al88.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_argb8888_premultiplied.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_i1.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_l8.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb565_swapped.su ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.cyclo ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.d ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.o ./Display_Driver/lvgl/src/draw/sw/blend/lv_draw_sw_blend_to_rgb888.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw-2f-blend

