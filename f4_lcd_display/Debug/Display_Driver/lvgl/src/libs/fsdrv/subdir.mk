################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.c \
../Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.c 

OBJS += \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.o \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.o 

C_DEPS += \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.d \
./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/libs/fsdrv/%.o Display_Driver/lvgl/src/libs/fsdrv/%.su Display_Driver/lvgl/src/libs/fsdrv/%.cyclo: ../Display_Driver/lvgl/src/libs/fsdrv/%.c Display_Driver/lvgl/src/libs/fsdrv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-fsdrv

clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-fsdrv:
	-$(RM) ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_cbfs.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_fatfs.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_frogfs.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_littlefs.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_memfs.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_posix.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_stdio.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_uefi.su ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.cyclo ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.d ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.o ./Display_Driver/lvgl/src/libs/fsdrv/lv_fs_win32.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-libs-2f-fsdrv

