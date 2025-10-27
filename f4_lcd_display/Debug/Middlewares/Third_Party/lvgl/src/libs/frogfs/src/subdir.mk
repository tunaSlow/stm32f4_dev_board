################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.c \
../Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.o \
./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.d \
./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/libs/frogfs/src/%.o Middlewares/Third_Party/lvgl/src/libs/frogfs/src/%.su Middlewares/Third_Party/lvgl/src/libs/frogfs/src/%.cyclo: ../Middlewares/Third_Party/lvgl/src/libs/frogfs/src/%.c Middlewares/Third_Party/lvgl/src/libs/frogfs/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App/lvgl_port" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.cyclo ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.d ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.o ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/decomp_raw.su ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.cyclo ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.d ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.o ./Middlewares/Third_Party/lvgl/src/libs/frogfs/src/frogfs.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-frogfs-2f-src

