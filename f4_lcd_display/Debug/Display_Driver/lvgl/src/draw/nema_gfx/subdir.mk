################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.c \
../Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.c 

OBJS += \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.o \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.d \
./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/nema_gfx/%.o Display_Driver/lvgl/src/draw/nema_gfx/%.su Display_Driver/lvgl/src/draw/nema_gfx/%.cyclo: ../Display_Driver/lvgl/src/draw/nema_gfx/%.c Display_Driver/lvgl/src/draw/nema_gfx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nema_gfx

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nema_gfx:
	-$(RM) ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_arc.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_border.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_fill.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_img.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_label.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_layer.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_line.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_stm32_hal.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_triangle.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_utils.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_draw_nema_gfx_vector.su ./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.cyclo ./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.d ./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.o ./Display_Driver/lvgl/src/draw/nema_gfx/lv_nema_gfx_path.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nema_gfx

