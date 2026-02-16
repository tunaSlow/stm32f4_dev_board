################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.c \
../Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.c 

OBJS += \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.o \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.d \
./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/renesas/dave2d/%.o Display_Driver/lvgl/src/draw/renesas/dave2d/%.su Display_Driver/lvgl/src/draw/renesas/dave2d/%.cyclo: ../Display_Driver/lvgl/src/draw/renesas/dave2d/%.c Display_Driver/lvgl/src/draw/renesas/dave2d/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-renesas-2f-dave2d

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-renesas-2f-dave2d:
	-$(RM) ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_arc.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_border.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_fill.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_image.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_label.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_line.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_mask_rectangle.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_triangle.su ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.cyclo ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.d ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.o ./Display_Driver/lvgl/src/draw/renesas/dave2d/lv_draw_dave2d_utils.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-renesas-2f-dave2d

