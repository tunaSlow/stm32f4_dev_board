################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.c \
../Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.c 

OBJS += \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.o \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.o 

C_DEPS += \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.d \
./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/drivers/nuttx/%.o Display_Driver/lvgl/src/drivers/nuttx/%.su Display_Driver/lvgl/src/drivers/nuttx/%.cyclo: ../Display_Driver/lvgl/src/drivers/nuttx/%.c Display_Driver/lvgl/src/drivers/nuttx/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-nuttx

clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-nuttx:
	-$(RM) ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_cache.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_entry.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_fbdev.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_image_cache.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_lcd.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_libuv.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_mouse.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_profiler.su ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.cyclo ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.d ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.o ./Display_Driver/lvgl/src/drivers/nuttx/lv_nuttx_touchscreen.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-nuttx

