################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.c \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.c \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.c \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.c \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.c \
../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.o \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.o \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.o \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.o \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.o \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.d \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.d \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.d \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.d \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.d \
./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/%.o Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/%.su Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/%.cyclo: ../Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/%.c Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-g2d

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-g2d:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_buf_g2d.su ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d.su ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_fill.su ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_draw_g2d_img.su ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_buf_map.su ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.cyclo ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.d ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.o ./Middlewares/Third_Party/lvgl/src/draw/nxp/g2d/lv_g2d_utils.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-draw-2f-nxp-2f-g2d

