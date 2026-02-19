################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/core/lv_group.c \
../Display_Driver/lvgl/src/core/lv_obj.c \
../Display_Driver/lvgl/src/core/lv_obj_class.c \
../Display_Driver/lvgl/src/core/lv_obj_draw.c \
../Display_Driver/lvgl/src/core/lv_obj_event.c \
../Display_Driver/lvgl/src/core/lv_obj_id_builtin.c \
../Display_Driver/lvgl/src/core/lv_obj_pos.c \
../Display_Driver/lvgl/src/core/lv_obj_property.c \
../Display_Driver/lvgl/src/core/lv_obj_scroll.c \
../Display_Driver/lvgl/src/core/lv_obj_style.c \
../Display_Driver/lvgl/src/core/lv_obj_style_gen.c \
../Display_Driver/lvgl/src/core/lv_obj_tree.c \
../Display_Driver/lvgl/src/core/lv_refr.c 

OBJS += \
./Display_Driver/lvgl/src/core/lv_group.o \
./Display_Driver/lvgl/src/core/lv_obj.o \
./Display_Driver/lvgl/src/core/lv_obj_class.o \
./Display_Driver/lvgl/src/core/lv_obj_draw.o \
./Display_Driver/lvgl/src/core/lv_obj_event.o \
./Display_Driver/lvgl/src/core/lv_obj_id_builtin.o \
./Display_Driver/lvgl/src/core/lv_obj_pos.o \
./Display_Driver/lvgl/src/core/lv_obj_property.o \
./Display_Driver/lvgl/src/core/lv_obj_scroll.o \
./Display_Driver/lvgl/src/core/lv_obj_style.o \
./Display_Driver/lvgl/src/core/lv_obj_style_gen.o \
./Display_Driver/lvgl/src/core/lv_obj_tree.o \
./Display_Driver/lvgl/src/core/lv_refr.o 

C_DEPS += \
./Display_Driver/lvgl/src/core/lv_group.d \
./Display_Driver/lvgl/src/core/lv_obj.d \
./Display_Driver/lvgl/src/core/lv_obj_class.d \
./Display_Driver/lvgl/src/core/lv_obj_draw.d \
./Display_Driver/lvgl/src/core/lv_obj_event.d \
./Display_Driver/lvgl/src/core/lv_obj_id_builtin.d \
./Display_Driver/lvgl/src/core/lv_obj_pos.d \
./Display_Driver/lvgl/src/core/lv_obj_property.d \
./Display_Driver/lvgl/src/core/lv_obj_scroll.d \
./Display_Driver/lvgl/src/core/lv_obj_style.d \
./Display_Driver/lvgl/src/core/lv_obj_style_gen.d \
./Display_Driver/lvgl/src/core/lv_obj_tree.d \
./Display_Driver/lvgl/src/core/lv_refr.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/core/%.o Display_Driver/lvgl/src/core/%.su Display_Driver/lvgl/src/core/%.cyclo: ../Display_Driver/lvgl/src/core/%.c Display_Driver/lvgl/src/core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-core

clean-Display_Driver-2f-lvgl-2f-src-2f-core:
	-$(RM) ./Display_Driver/lvgl/src/core/lv_group.cyclo ./Display_Driver/lvgl/src/core/lv_group.d ./Display_Driver/lvgl/src/core/lv_group.o ./Display_Driver/lvgl/src/core/lv_group.su ./Display_Driver/lvgl/src/core/lv_obj.cyclo ./Display_Driver/lvgl/src/core/lv_obj.d ./Display_Driver/lvgl/src/core/lv_obj.o ./Display_Driver/lvgl/src/core/lv_obj.su ./Display_Driver/lvgl/src/core/lv_obj_class.cyclo ./Display_Driver/lvgl/src/core/lv_obj_class.d ./Display_Driver/lvgl/src/core/lv_obj_class.o ./Display_Driver/lvgl/src/core/lv_obj_class.su ./Display_Driver/lvgl/src/core/lv_obj_draw.cyclo ./Display_Driver/lvgl/src/core/lv_obj_draw.d ./Display_Driver/lvgl/src/core/lv_obj_draw.o ./Display_Driver/lvgl/src/core/lv_obj_draw.su ./Display_Driver/lvgl/src/core/lv_obj_event.cyclo ./Display_Driver/lvgl/src/core/lv_obj_event.d ./Display_Driver/lvgl/src/core/lv_obj_event.o ./Display_Driver/lvgl/src/core/lv_obj_event.su ./Display_Driver/lvgl/src/core/lv_obj_id_builtin.cyclo ./Display_Driver/lvgl/src/core/lv_obj_id_builtin.d ./Display_Driver/lvgl/src/core/lv_obj_id_builtin.o ./Display_Driver/lvgl/src/core/lv_obj_id_builtin.su ./Display_Driver/lvgl/src/core/lv_obj_pos.cyclo ./Display_Driver/lvgl/src/core/lv_obj_pos.d ./Display_Driver/lvgl/src/core/lv_obj_pos.o ./Display_Driver/lvgl/src/core/lv_obj_pos.su ./Display_Driver/lvgl/src/core/lv_obj_property.cyclo ./Display_Driver/lvgl/src/core/lv_obj_property.d ./Display_Driver/lvgl/src/core/lv_obj_property.o ./Display_Driver/lvgl/src/core/lv_obj_property.su ./Display_Driver/lvgl/src/core/lv_obj_scroll.cyclo ./Display_Driver/lvgl/src/core/lv_obj_scroll.d ./Display_Driver/lvgl/src/core/lv_obj_scroll.o ./Display_Driver/lvgl/src/core/lv_obj_scroll.su ./Display_Driver/lvgl/src/core/lv_obj_style.cyclo ./Display_Driver/lvgl/src/core/lv_obj_style.d ./Display_Driver/lvgl/src/core/lv_obj_style.o ./Display_Driver/lvgl/src/core/lv_obj_style.su ./Display_Driver/lvgl/src/core/lv_obj_style_gen.cyclo ./Display_Driver/lvgl/src/core/lv_obj_style_gen.d ./Display_Driver/lvgl/src/core/lv_obj_style_gen.o ./Display_Driver/lvgl/src/core/lv_obj_style_gen.su ./Display_Driver/lvgl/src/core/lv_obj_tree.cyclo ./Display_Driver/lvgl/src/core/lv_obj_tree.d ./Display_Driver/lvgl/src/core/lv_obj_tree.o ./Display_Driver/lvgl/src/core/lv_obj_tree.su ./Display_Driver/lvgl/src/core/lv_refr.cyclo ./Display_Driver/lvgl/src/core/lv_refr.d ./Display_Driver/lvgl/src/core/lv_refr.o ./Display_Driver/lvgl/src/core/lv_refr.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-core

