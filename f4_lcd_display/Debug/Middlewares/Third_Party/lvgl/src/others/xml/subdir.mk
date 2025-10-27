################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.c \
../Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.o \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.d \
./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/others/xml/%.o Middlewares/Third_Party/lvgl/src/others/xml/%.su Middlewares/Third_Party/lvgl/src/others/xml/%.cyclo: ../Middlewares/Third_Party/lvgl/src/others/xml/%.c Middlewares/Third_Party/lvgl/src/others/xml/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App/lvgl_port" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-xml

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-xml:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_base_types.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_component.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_load.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_parser.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_style.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_test.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_translation.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_update.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_utils.su ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.cyclo ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.d ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.o ./Middlewares/Third_Party/lvgl/src/others/xml/lv_xml_widget.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-xml

