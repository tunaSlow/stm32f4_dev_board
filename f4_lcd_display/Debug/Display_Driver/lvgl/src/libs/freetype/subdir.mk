################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/libs/freetype/lv_freetype.c \
../Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.c \
../Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.c \
../Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.c \
../Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.c 

OBJS += \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype.o \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.o \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.o \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.o \
./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.o 

C_DEPS += \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype.d \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.d \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.d \
./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.d \
./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/libs/freetype/%.o Display_Driver/lvgl/src/libs/freetype/%.su Display_Driver/lvgl/src/libs/freetype/%.cyclo: ../Display_Driver/lvgl/src/libs/freetype/%.c Display_Driver/lvgl/src/libs/freetype/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-freetype

clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-freetype:
	-$(RM) ./Display_Driver/lvgl/src/libs/freetype/lv_freetype.cyclo ./Display_Driver/lvgl/src/libs/freetype/lv_freetype.d ./Display_Driver/lvgl/src/libs/freetype/lv_freetype.o ./Display_Driver/lvgl/src/libs/freetype/lv_freetype.su ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.cyclo ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.d ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.o ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_glyph.su ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.cyclo ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.d ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.o ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_image.su ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.cyclo ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.d ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.o ./Display_Driver/lvgl/src/libs/freetype/lv_freetype_outline.su ./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.cyclo ./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.d ./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.o ./Display_Driver/lvgl/src/libs/freetype/lv_ftsystem.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-freetype

