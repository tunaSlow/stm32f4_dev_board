################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.c \
../Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.o \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.d \
./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/drivers/wayland/%.o Middlewares/Third_Party/lvgl/src/drivers/wayland/%.su Middlewares/Third_Party/lvgl/src/drivers/wayland/%.cyclo: ../Middlewares/Third_Party/lvgl/src/drivers/wayland/%.c Middlewares/Third_Party/lvgl/src/drivers/wayland/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App/lvgl_port" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-drivers-2f-wayland

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-drivers-2f-wayland:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wayland_smm.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_cache.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_dmabuf.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_keyboard.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_pointer_axis.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_seat.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_shm.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_touch.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_window_decorations.su ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.cyclo ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.d ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.o ./Middlewares/Third_Party/lvgl/src/drivers/wayland/lv_wl_xdg_shell.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-drivers-2f-wayland

