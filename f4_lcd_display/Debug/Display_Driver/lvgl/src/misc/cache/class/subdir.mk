################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.c \
../Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.c \
../Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.c 

OBJS += \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.o \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.o \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.o 

C_DEPS += \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.d \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.d \
./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/misc/cache/class/%.o Display_Driver/lvgl/src/misc/cache/class/%.su Display_Driver/lvgl/src/misc/cache/class/%.cyclo: ../Display_Driver/lvgl/src/misc/cache/class/%.c Display_Driver/lvgl/src/misc/cache/class/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class

clean-Display_Driver-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class:
	-$(RM) ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.cyclo ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.d ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.o ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_ll.su ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.cyclo ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.d ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.o ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_lru_rb.su ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.cyclo ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.d ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.o ./Display_Driver/lvgl/src/misc/cache/class/lv_cache_sc_da.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-misc-2f-cache-2f-class

