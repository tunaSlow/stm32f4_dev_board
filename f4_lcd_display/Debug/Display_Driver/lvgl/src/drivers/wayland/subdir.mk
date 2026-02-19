################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/drivers/wayland/lv_wayland.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.c \
../Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.c 

OBJS += \
./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.o \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.o 

C_DEPS += \
./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.d \
./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/drivers/wayland/%.o Display_Driver/lvgl/src/drivers/wayland/%.su Display_Driver/lvgl/src/drivers/wayland/%.cyclo: ../Display_Driver/lvgl/src/drivers/wayland/%.c Display_Driver/lvgl/src/drivers/wayland/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitKraken/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-wayland

clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-wayland:
	-$(RM) ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wayland_smm.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_cache.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_dmabuf.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_keyboard.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_pointer_axis.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_seat.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_shm.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_touch.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_window_decorations.su ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.cyclo ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.d ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.o ./Display_Driver/lvgl/src/drivers/wayland/lv_wl_xdg_shell.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-drivers-2f-wayland

