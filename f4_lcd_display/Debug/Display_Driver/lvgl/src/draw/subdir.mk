################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/lv_draw.c \
../Display_Driver/lvgl/src/draw/lv_draw_3d.c \
../Display_Driver/lvgl/src/draw/lv_draw_arc.c \
../Display_Driver/lvgl/src/draw/lv_draw_buf.c \
../Display_Driver/lvgl/src/draw/lv_draw_image.c \
../Display_Driver/lvgl/src/draw/lv_draw_label.c \
../Display_Driver/lvgl/src/draw/lv_draw_line.c \
../Display_Driver/lvgl/src/draw/lv_draw_mask.c \
../Display_Driver/lvgl/src/draw/lv_draw_rect.c \
../Display_Driver/lvgl/src/draw/lv_draw_triangle.c \
../Display_Driver/lvgl/src/draw/lv_draw_vector.c \
../Display_Driver/lvgl/src/draw/lv_image_decoder.c 

OBJS += \
./Display_Driver/lvgl/src/draw/lv_draw.o \
./Display_Driver/lvgl/src/draw/lv_draw_3d.o \
./Display_Driver/lvgl/src/draw/lv_draw_arc.o \
./Display_Driver/lvgl/src/draw/lv_draw_buf.o \
./Display_Driver/lvgl/src/draw/lv_draw_image.o \
./Display_Driver/lvgl/src/draw/lv_draw_label.o \
./Display_Driver/lvgl/src/draw/lv_draw_line.o \
./Display_Driver/lvgl/src/draw/lv_draw_mask.o \
./Display_Driver/lvgl/src/draw/lv_draw_rect.o \
./Display_Driver/lvgl/src/draw/lv_draw_triangle.o \
./Display_Driver/lvgl/src/draw/lv_draw_vector.o \
./Display_Driver/lvgl/src/draw/lv_image_decoder.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/lv_draw.d \
./Display_Driver/lvgl/src/draw/lv_draw_3d.d \
./Display_Driver/lvgl/src/draw/lv_draw_arc.d \
./Display_Driver/lvgl/src/draw/lv_draw_buf.d \
./Display_Driver/lvgl/src/draw/lv_draw_image.d \
./Display_Driver/lvgl/src/draw/lv_draw_label.d \
./Display_Driver/lvgl/src/draw/lv_draw_line.d \
./Display_Driver/lvgl/src/draw/lv_draw_mask.d \
./Display_Driver/lvgl/src/draw/lv_draw_rect.d \
./Display_Driver/lvgl/src/draw/lv_draw_triangle.d \
./Display_Driver/lvgl/src/draw/lv_draw_vector.d \
./Display_Driver/lvgl/src/draw/lv_image_decoder.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/%.o Display_Driver/lvgl/src/draw/%.su Display_Driver/lvgl/src/draw/%.cyclo: ../Display_Driver/lvgl/src/draw/%.c Display_Driver/lvgl/src/draw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw

clean-Display_Driver-2f-lvgl-2f-src-2f-draw:
	-$(RM) ./Display_Driver/lvgl/src/draw/lv_draw.cyclo ./Display_Driver/lvgl/src/draw/lv_draw.d ./Display_Driver/lvgl/src/draw/lv_draw.o ./Display_Driver/lvgl/src/draw/lv_draw.su ./Display_Driver/lvgl/src/draw/lv_draw_3d.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_3d.d ./Display_Driver/lvgl/src/draw/lv_draw_3d.o ./Display_Driver/lvgl/src/draw/lv_draw_3d.su ./Display_Driver/lvgl/src/draw/lv_draw_arc.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_arc.d ./Display_Driver/lvgl/src/draw/lv_draw_arc.o ./Display_Driver/lvgl/src/draw/lv_draw_arc.su ./Display_Driver/lvgl/src/draw/lv_draw_buf.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_buf.d ./Display_Driver/lvgl/src/draw/lv_draw_buf.o ./Display_Driver/lvgl/src/draw/lv_draw_buf.su ./Display_Driver/lvgl/src/draw/lv_draw_image.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_image.d ./Display_Driver/lvgl/src/draw/lv_draw_image.o ./Display_Driver/lvgl/src/draw/lv_draw_image.su ./Display_Driver/lvgl/src/draw/lv_draw_label.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_label.d ./Display_Driver/lvgl/src/draw/lv_draw_label.o ./Display_Driver/lvgl/src/draw/lv_draw_label.su ./Display_Driver/lvgl/src/draw/lv_draw_line.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_line.d ./Display_Driver/lvgl/src/draw/lv_draw_line.o ./Display_Driver/lvgl/src/draw/lv_draw_line.su ./Display_Driver/lvgl/src/draw/lv_draw_mask.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_mask.d ./Display_Driver/lvgl/src/draw/lv_draw_mask.o ./Display_Driver/lvgl/src/draw/lv_draw_mask.su ./Display_Driver/lvgl/src/draw/lv_draw_rect.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_rect.d ./Display_Driver/lvgl/src/draw/lv_draw_rect.o ./Display_Driver/lvgl/src/draw/lv_draw_rect.su ./Display_Driver/lvgl/src/draw/lv_draw_triangle.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_triangle.d ./Display_Driver/lvgl/src/draw/lv_draw_triangle.o ./Display_Driver/lvgl/src/draw/lv_draw_triangle.su ./Display_Driver/lvgl/src/draw/lv_draw_vector.cyclo ./Display_Driver/lvgl/src/draw/lv_draw_vector.d ./Display_Driver/lvgl/src/draw/lv_draw_vector.o ./Display_Driver/lvgl/src/draw/lv_draw_vector.su ./Display_Driver/lvgl/src/draw/lv_image_decoder.cyclo ./Display_Driver/lvgl/src/draw/lv_image_decoder.d ./Display_Driver/lvgl/src/draw/lv_image_decoder.o ./Display_Driver/lvgl/src/draw/lv_image_decoder.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw

