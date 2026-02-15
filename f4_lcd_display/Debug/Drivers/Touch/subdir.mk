################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Touch/touch_driver.c 

OBJS += \
./Drivers/Touch/touch_driver.o 

C_DEPS += \
./Drivers/Touch/touch_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Touch/%.o Drivers/Touch/%.su Drivers/Touch/%.cyclo: ../Drivers/Touch/%.c Drivers/Touch/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/App" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Touch

clean-Drivers-2f-Touch:
	-$(RM) ./Drivers/Touch/touch_driver.cyclo ./Drivers/Touch/touch_driver.d ./Drivers/Touch/touch_driver.o ./Drivers/Touch/touch_driver.su

.PHONY: clean-Drivers-2f-Touch

