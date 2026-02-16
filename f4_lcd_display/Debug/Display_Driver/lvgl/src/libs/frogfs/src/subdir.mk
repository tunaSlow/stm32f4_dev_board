################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.c \
../Display_Driver/lvgl/src/libs/frogfs/src/frogfs.c 

OBJS += \
./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.o \
./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.o 

C_DEPS += \
./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.d \
./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/libs/frogfs/src/%.o Display_Driver/lvgl/src/libs/frogfs/src/%.su Display_Driver/lvgl/src/libs/frogfs/src/%.cyclo: ../Display_Driver/lvgl/src/libs/frogfs/src/%.c Display_Driver/lvgl/src/libs/frogfs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src

clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src:
	-$(RM) ./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.cyclo ./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.d ./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.o ./Display_Driver/lvgl/src/libs/frogfs/src/decomp_raw.su ./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.cyclo ./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.d ./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.o ./Display_Driver/lvgl/src/libs/frogfs/src/frogfs.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src

