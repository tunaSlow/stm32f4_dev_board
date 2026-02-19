################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.c \
../Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.c 

OBJS += \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.o \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.d \
./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/sw/%.o Display_Driver/lvgl/src/draw/sw/%.su Display_Driver/lvgl/src/draw/sw/%.cyclo: ../Display_Driver/lvgl/src/draw/sw/%.c Display_Driver/lvgl/src/draw/sw/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw:
	-$(RM) ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_arc.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_border.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_box_shadow.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_fill.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_grad.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_img.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_letter.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_line.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_mask_rect.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_transform.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_triangle.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_utils.su ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.cyclo ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.d ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.o ./Display_Driver/lvgl/src/draw/sw/lv_draw_sw_vector.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-sw

