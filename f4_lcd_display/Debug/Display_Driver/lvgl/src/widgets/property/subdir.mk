################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_image_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_label_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_obj_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_roller_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_slider_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_style_properties.c \
../Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.c 

OBJS += \
./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_image_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_label_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_style_properties.o \
./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.o 

C_DEPS += \
./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_image_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_label_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_style_properties.d \
./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/widgets/property/%.o Display_Driver/lvgl/src/widgets/property/%.su Display_Driver/lvgl/src/widgets/property/%.cyclo: ../Display_Driver/lvgl/src/widgets/property/%.c Display_Driver/lvgl/src/widgets/property/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-property

clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-property:
	-$(RM) ./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_animimage_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_dropdown_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_image_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_image_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_image_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_image_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_keyboard_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_label_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_label_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_label_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_label_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_obj_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_roller_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_slider_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_style_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_style_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_style_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_style_properties.su ./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.cyclo ./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.d ./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.o ./Display_Driver/lvgl/src/widgets/property/lv_textarea_properties.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-property

