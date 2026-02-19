################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.c \
../Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.c \
../Display_Driver/lvgl/src/draw/eve/lv_eve.c 

OBJS += \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.o \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.o \
./Display_Driver/lvgl/src/draw/eve/lv_eve.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.d \
./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.d \
./Display_Driver/lvgl/src/draw/eve/lv_eve.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/eve/%.o Display_Driver/lvgl/src/draw/eve/%.su Display_Driver/lvgl/src/draw/eve/%.cyclo: ../Display_Driver/lvgl/src/draw/eve/%.c Display_Driver/lvgl/src/draw/eve/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-eve

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-eve:
	-$(RM) ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_arc.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_fill.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_image.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_letter.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_line.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_ram_g.su ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.d ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.o ./Display_Driver/lvgl/src/draw/eve/lv_draw_eve_triangle.su ./Display_Driver/lvgl/src/draw/eve/lv_eve.cyclo ./Display_Driver/lvgl/src/draw/eve/lv_eve.d ./Display_Driver/lvgl/src/draw/eve/lv_eve.o ./Display_Driver/lvgl/src/draw/eve/lv_eve.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-eve

