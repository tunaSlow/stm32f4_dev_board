################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/others/file_explorer/%.o Middlewares/Third_Party/lvgl/src/others/file_explorer/%.su Middlewares/Third_Party/lvgl/src/others/file_explorer/%.cyclo: ../Middlewares/Third_Party/lvgl/src/others/file_explorer/%.c Middlewares/Third_Party/lvgl/src/others/file_explorer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/App" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-file_explorer

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-file_explorer:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.cyclo ./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.d ./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.o ./Middlewares/Third_Party/lvgl/src/others/file_explorer/lv_file_explorer.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-file_explorer

