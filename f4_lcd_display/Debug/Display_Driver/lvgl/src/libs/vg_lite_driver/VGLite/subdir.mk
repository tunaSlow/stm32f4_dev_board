################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.c \
../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.c \
../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.c \
../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.c \
../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.c 

OBJS += \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.o \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.o \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.o \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.o \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.o 

C_DEPS += \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.d \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.d \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.d \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.d \
./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/%.o Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/%.su Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/%.cyclo: ../Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/%.c Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLite

clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLite:
	-$(RM) ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.cyclo ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.d ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.o ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite.su ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.cyclo ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.d ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.o ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_image.su ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.cyclo ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.d ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.o ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_matrix.su ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.cyclo ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.d ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.o ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_path.su ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.cyclo ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.d ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.o ./Display_Driver/lvgl/src/libs/vg_lite_driver/VGLite/vg_lite_stroke.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-vg_lite_driver-2f-VGLite

