################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.c \
../Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.c 

OBJS += \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.o \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.o 

C_DEPS += \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.d \
./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/draw/nxp/pxp/%.o Display_Driver/lvgl/src/draw/nxp/pxp/%.su Display_Driver/lvgl/src/draw/nxp/pxp/%.cyclo: ../Display_Driver/lvgl/src/draw/nxp/pxp/%.c Display_Driver/lvgl/src/draw/nxp/pxp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-pxp

clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-pxp:
	-$(RM) ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_buf_pxp.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_fill.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_img.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_draw_pxp_layer.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_cfg.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_osa.su ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.cyclo ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.d ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.o ./Display_Driver/lvgl/src/draw/nxp/pxp/lv_pxp_utils.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-pxp

