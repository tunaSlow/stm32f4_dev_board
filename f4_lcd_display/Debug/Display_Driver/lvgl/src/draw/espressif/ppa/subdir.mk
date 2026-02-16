################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.c \
../Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.c \
../Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.c \
../Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.c 

OBJS += \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.o \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.o \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.o \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.d \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.d \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.d \
./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/espressif/ppa/%.o Display_Driver/lvgl/src/draw/espressif/ppa/%.su Display_Driver/lvgl/src/draw/espressif/ppa/%.cyclo: ../Display_Driver/lvgl/src/draw/espressif/ppa/%.c Display_Driver/lvgl/src/draw/espressif/ppa/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-espressif-2f-ppa

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-espressif-2f-ppa:
	-$(RM) ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.cyclo ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.d ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.o ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa.su ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.cyclo ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.d ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.o ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_buf.su ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.cyclo ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.d ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.o ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_fill.su ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.cyclo ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.d ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.o ./Display_Driver/lvgl/src/draw/espressif/ppa/lv_draw_ppa_img.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-espressif-2f-ppa

