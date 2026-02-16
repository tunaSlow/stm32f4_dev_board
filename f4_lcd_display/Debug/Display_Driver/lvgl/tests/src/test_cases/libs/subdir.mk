################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.c \
../Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.c 

OBJS += \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.o \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.o 

C_DEPS += \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.d \
./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/tests/src/test_cases/libs/%.o Display_Driver/lvgl/tests/src/test_cases/libs/%.su Display_Driver/lvgl/tests/src/test_cases/libs/%.cyclo: ../Display_Driver/lvgl/tests/src/test_cases/libs/%.c Display_Driver/lvgl/tests/src/test_cases/libs/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases-2f-libs

clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases-2f-libs:
	-$(RM) ./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_barcode.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bin_decoder.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_bmp.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_ffmpeg.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_font_stress.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_freetype.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libjpeg_turbo.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_libpng.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_lodepng.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_memmove.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_qrcode.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_svg_decoder.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tiny_ttf.su ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.cyclo ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.d ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.o ./Display_Driver/lvgl/tests/src/test_cases/libs/test_tjpgd.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases-2f-libs

