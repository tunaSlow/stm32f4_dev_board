################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.c \
../Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.c 

OBJS += \
./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.o \
./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.o 

C_DEPS += \
./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.d \
./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/drivers/opengles/opengl_shader/%.o Display_Driver/lvgl/src/drivers/opengles/opengl_shader/%.su Display_Driver/lvgl/src/drivers/opengles/opengl_shader/%.cyclo: ../Display_Driver/lvgl/src/drivers/opengles/opengl_shader/%.c Display_Driver/lvgl/src/drivers/opengles/opengl_shader/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-opengl_shader

clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-opengl_shader:
	-$(RM) ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.cyclo ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.d ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.o ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_manager.su ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.cyclo ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.d ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.o ./Display_Driver/lvgl/src/drivers/opengles/opengl_shader/lv_opengl_shader_program.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-opengles-2f-opengl_shader

