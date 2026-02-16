################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.c \
../Display_Driver/lvgl/src/libs/qrcode/qrcodegen.c 

OBJS += \
./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.o \
./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.o 

C_DEPS += \
./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.d \
./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/libs/qrcode/%.o Display_Driver/lvgl/src/libs/qrcode/%.su Display_Driver/lvgl/src/libs/qrcode/%.cyclo: ../Display_Driver/lvgl/src/libs/qrcode/%.c Display_Driver/lvgl/src/libs/qrcode/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-qrcode

clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-qrcode:
	-$(RM) ./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.cyclo ./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.d ./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.o ./Display_Driver/lvgl/src/libs/qrcode/lv_qrcode.su ./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.cyclo ./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.d ./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.o ./Display_Driver/lvgl/src/libs/qrcode/qrcodegen.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-qrcode

