################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.c \
../Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.c 

OBJS += \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.o \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.d \
./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/vg_lite/%.o Display_Driver/lvgl/src/draw/vg_lite/%.su Display_Driver/lvgl/src/draw/vg_lite/%.cyclo: ../Display_Driver/lvgl/src/draw/vg_lite/%.c Display_Driver/lvgl/src/draw/vg_lite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-vg_lite

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-vg_lite:
	-$(RM) ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_buf_vg_lite.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_arc.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_border.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_box_shadow.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_fill.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_img.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_label.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_layer.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_line.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_mask_rect.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_triangle.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_draw_vg_lite_vector.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_decoder.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_grad.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_math.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_path.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_pending.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_stroke.su ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.cyclo ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.d ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.o ./Display_Driver/lvgl/src/draw/vg_lite/lv_vg_lite_utils.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-vg_lite

