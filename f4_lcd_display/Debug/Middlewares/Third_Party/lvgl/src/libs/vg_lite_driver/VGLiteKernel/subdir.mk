################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/%.o Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/%.su Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/%.cyclo: ../Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/%.c Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App/lvgl_port" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLiteKernel

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLiteKernel:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.cyclo ./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.d ./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.o ./Middlewares/Third_Party/lvgl/src/libs/vg_lite_driver/VGLiteKernel/vg_lite_kernel.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLiteKernel

