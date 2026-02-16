################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/others/xml/lv_xml.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_base_types.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_component.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_load.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_parser.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_style.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_test.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_translation.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_update.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_utils.c \
../Display_Driver/lvgl/src/others/xml/lv_xml_widget.c 

OBJS += \
./Display_Driver/lvgl/src/others/xml/lv_xml.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_component.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_load.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_parser.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_style.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_test.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_translation.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_update.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_utils.o \
./Display_Driver/lvgl/src/others/xml/lv_xml_widget.o 

C_DEPS += \
./Display_Driver/lvgl/src/others/xml/lv_xml.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_component.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_load.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_parser.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_style.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_test.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_translation.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_update.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_utils.d \
./Display_Driver/lvgl/src/others/xml/lv_xml_widget.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/others/xml/%.o Display_Driver/lvgl/src/others/xml/%.su Display_Driver/lvgl/src/others/xml/%.cyclo: ../Display_Driver/lvgl/src/others/xml/%.c Display_Driver/lvgl/src/others/xml/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-xml

clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-xml:
	-$(RM) ./Display_Driver/lvgl/src/others/xml/lv_xml.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml.d ./Display_Driver/lvgl/src/others/xml/lv_xml.o ./Display_Driver/lvgl/src/others/xml/lv_xml.su ./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.d ./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.o ./Display_Driver/lvgl/src/others/xml/lv_xml_base_types.su ./Display_Driver/lvgl/src/others/xml/lv_xml_component.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_component.d ./Display_Driver/lvgl/src/others/xml/lv_xml_component.o ./Display_Driver/lvgl/src/others/xml/lv_xml_component.su ./Display_Driver/lvgl/src/others/xml/lv_xml_load.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_load.d ./Display_Driver/lvgl/src/others/xml/lv_xml_load.o ./Display_Driver/lvgl/src/others/xml/lv_xml_load.su ./Display_Driver/lvgl/src/others/xml/lv_xml_parser.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_parser.d ./Display_Driver/lvgl/src/others/xml/lv_xml_parser.o ./Display_Driver/lvgl/src/others/xml/lv_xml_parser.su ./Display_Driver/lvgl/src/others/xml/lv_xml_style.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_style.d ./Display_Driver/lvgl/src/others/xml/lv_xml_style.o ./Display_Driver/lvgl/src/others/xml/lv_xml_style.su ./Display_Driver/lvgl/src/others/xml/lv_xml_test.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_test.d ./Display_Driver/lvgl/src/others/xml/lv_xml_test.o ./Display_Driver/lvgl/src/others/xml/lv_xml_test.su ./Display_Driver/lvgl/src/others/xml/lv_xml_translation.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_translation.d ./Display_Driver/lvgl/src/others/xml/lv_xml_translation.o ./Display_Driver/lvgl/src/others/xml/lv_xml_translation.su ./Display_Driver/lvgl/src/others/xml/lv_xml_update.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_update.d ./Display_Driver/lvgl/src/others/xml/lv_xml_update.o ./Display_Driver/lvgl/src/others/xml/lv_xml_update.su ./Display_Driver/lvgl/src/others/xml/lv_xml_utils.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_utils.d ./Display_Driver/lvgl/src/others/xml/lv_xml_utils.o ./Display_Driver/lvgl/src/others/xml/lv_xml_utils.su ./Display_Driver/lvgl/src/others/xml/lv_xml_widget.cyclo ./Display_Driver/lvgl/src/others/xml/lv_xml_widget.d ./Display_Driver/lvgl/src/others/xml/lv_xml_widget.o ./Display_Driver/lvgl/src/others/xml/lv_xml_widget.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-xml

