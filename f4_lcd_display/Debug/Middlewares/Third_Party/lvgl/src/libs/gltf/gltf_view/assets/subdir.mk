################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.c \
../Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.o \
./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.d \
./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/%.o Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/%.su Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/%.cyclo: ../Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/%.c Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/App" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.cyclo ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.d ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.o ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/chromatic.su ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.cyclo ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.d ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.o ./Middlewares/Third_Party/lvgl/src/libs/gltf/gltf_view/assets/lv_gltf_view_shader.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-gltf-2f-gltf_view-2f-assets

