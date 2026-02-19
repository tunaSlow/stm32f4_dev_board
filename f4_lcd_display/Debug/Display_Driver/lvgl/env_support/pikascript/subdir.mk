################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lvgl.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.c \
../Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.c 

OBJS += \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.o \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.o 

C_DEPS += \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.d \
./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/env_support/pikascript/%.o Display_Driver/lvgl/env_support/pikascript/%.su Display_Driver/lvgl/env_support/pikascript/%.cyclo: ../Display_Driver/lvgl/env_support/pikascript/%.c Display_Driver/lvgl/env_support/pikascript/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-env_support-2f-pikascript

clean-Display_Driver-2f-lvgl-2f-env_support-2f-pikascript:
	-$(RM) ./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.d ./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.o ./Display_Driver/lvgl/env_support/pikascript/pika_lv_point_t.su ./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.d ./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.o ./Display_Driver/lvgl/env_support/pikascript/pika_lv_timer_t.su ./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.d ./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.o ./Display_Driver/lvgl/env_support/pikascript/pika_lv_wegit.su ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.d ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.o ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl.su ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.d ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.o ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_indev_t.su ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.d ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.o ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_event.su ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.d ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.o ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_obj.su ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.cyclo ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.d ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.o ./Display_Driver/lvgl/env_support/pikascript/pika_lvgl_lv_style_t.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-env_support-2f-pikascript

