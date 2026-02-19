################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ui/screens/ui_Screen1.c 

OBJS += \
./Core/Src/ui/screens/ui_Screen1.o 

C_DEPS += \
./Core/Src/ui/screens/ui_Screen1.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/ui/screens/%.o Core/Src/ui/screens/%.su Core/Src/ui/screens/%.cyclo: ../Core/Src/ui/screens/%.c Core/Src/ui/screens/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-ui-2f-screens

clean-Core-2f-Src-2f-ui-2f-screens:
	-$(RM) ./Core/Src/ui/screens/ui_Screen1.cyclo ./Core/Src/ui/screens/ui_Screen1.d ./Core/Src/ui/screens/ui_Screen1.o ./Core/Src/ui/screens/ui_Screen1.su

.PHONY: clean-Core-2f-Src-2f-ui-2f-screens

