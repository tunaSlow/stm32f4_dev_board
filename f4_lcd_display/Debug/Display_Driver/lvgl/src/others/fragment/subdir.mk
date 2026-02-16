################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/others/fragment/lv_fragment.c \
../Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.c 

OBJS += \
./Display_Driver/lvgl/src/others/fragment/lv_fragment.o \
./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.o 

C_DEPS += \
./Display_Driver/lvgl/src/others/fragment/lv_fragment.d \
./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/others/fragment/%.o Display_Driver/lvgl/src/others/fragment/%.su Display_Driver/lvgl/src/others/fragment/%.cyclo: ../Display_Driver/lvgl/src/others/fragment/%.c Display_Driver/lvgl/src/others/fragment/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-fragment

clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-fragment:
	-$(RM) ./Display_Driver/lvgl/src/others/fragment/lv_fragment.cyclo ./Display_Driver/lvgl/src/others/fragment/lv_fragment.d ./Display_Driver/lvgl/src/others/fragment/lv_fragment.o ./Display_Driver/lvgl/src/others/fragment/lv_fragment.su ./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.cyclo ./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.d ./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.o ./Display_Driver/lvgl/src/others/fragment/lv_fragment_manager.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-others-2f-fragment

