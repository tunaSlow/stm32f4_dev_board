################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.c \
../Display_Driver/lvgl/src/osal/lv_freertos.c \
../Display_Driver/lvgl/src/osal/lv_linux.c \
../Display_Driver/lvgl/src/osal/lv_mqx.c \
../Display_Driver/lvgl/src/osal/lv_os.c \
../Display_Driver/lvgl/src/osal/lv_os_none.c \
../Display_Driver/lvgl/src/osal/lv_pthread.c \
../Display_Driver/lvgl/src/osal/lv_rtthread.c \
../Display_Driver/lvgl/src/osal/lv_sdl2.c \
../Display_Driver/lvgl/src/osal/lv_windows.c 

OBJS += \
./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.o \
./Display_Driver/lvgl/src/osal/lv_freertos.o \
./Display_Driver/lvgl/src/osal/lv_linux.o \
./Display_Driver/lvgl/src/osal/lv_mqx.o \
./Display_Driver/lvgl/src/osal/lv_os.o \
./Display_Driver/lvgl/src/osal/lv_os_none.o \
./Display_Driver/lvgl/src/osal/lv_pthread.o \
./Display_Driver/lvgl/src/osal/lv_rtthread.o \
./Display_Driver/lvgl/src/osal/lv_sdl2.o \
./Display_Driver/lvgl/src/osal/lv_windows.o 

C_DEPS += \
./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.d \
./Display_Driver/lvgl/src/osal/lv_freertos.d \
./Display_Driver/lvgl/src/osal/lv_linux.d \
./Display_Driver/lvgl/src/osal/lv_mqx.d \
./Display_Driver/lvgl/src/osal/lv_os.d \
./Display_Driver/lvgl/src/osal/lv_os_none.d \
./Display_Driver/lvgl/src/osal/lv_pthread.d \
./Display_Driver/lvgl/src/osal/lv_rtthread.d \
./Display_Driver/lvgl/src/osal/lv_sdl2.d \
./Display_Driver/lvgl/src/osal/lv_windows.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/osal/%.o Display_Driver/lvgl/src/osal/%.su Display_Driver/lvgl/src/osal/%.cyclo: ../Display_Driver/lvgl/src/osal/%.c Display_Driver/lvgl/src/osal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-osal

clean-Display_Driver-2f-lvgl-2f-src-2f-osal:
	-$(RM) ./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.cyclo ./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.d ./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.o ./Display_Driver/lvgl/src/osal/lv_cmsis_rtos2.su ./Display_Driver/lvgl/src/osal/lv_freertos.cyclo ./Display_Driver/lvgl/src/osal/lv_freertos.d ./Display_Driver/lvgl/src/osal/lv_freertos.o ./Display_Driver/lvgl/src/osal/lv_freertos.su ./Display_Driver/lvgl/src/osal/lv_linux.cyclo ./Display_Driver/lvgl/src/osal/lv_linux.d ./Display_Driver/lvgl/src/osal/lv_linux.o ./Display_Driver/lvgl/src/osal/lv_linux.su ./Display_Driver/lvgl/src/osal/lv_mqx.cyclo ./Display_Driver/lvgl/src/osal/lv_mqx.d ./Display_Driver/lvgl/src/osal/lv_mqx.o ./Display_Driver/lvgl/src/osal/lv_mqx.su ./Display_Driver/lvgl/src/osal/lv_os.cyclo ./Display_Driver/lvgl/src/osal/lv_os.d ./Display_Driver/lvgl/src/osal/lv_os.o ./Display_Driver/lvgl/src/osal/lv_os.su ./Display_Driver/lvgl/src/osal/lv_os_none.cyclo ./Display_Driver/lvgl/src/osal/lv_os_none.d ./Display_Driver/lvgl/src/osal/lv_os_none.o ./Display_Driver/lvgl/src/osal/lv_os_none.su ./Display_Driver/lvgl/src/osal/lv_pthread.cyclo ./Display_Driver/lvgl/src/osal/lv_pthread.d ./Display_Driver/lvgl/src/osal/lv_pthread.o ./Display_Driver/lvgl/src/osal/lv_pthread.su ./Display_Driver/lvgl/src/osal/lv_rtthread.cyclo ./Display_Driver/lvgl/src/osal/lv_rtthread.d ./Display_Driver/lvgl/src/osal/lv_rtthread.o ./Display_Driver/lvgl/src/osal/lv_rtthread.su ./Display_Driver/lvgl/src/osal/lv_sdl2.cyclo ./Display_Driver/lvgl/src/osal/lv_sdl2.d ./Display_Driver/lvgl/src/osal/lv_sdl2.o ./Display_Driver/lvgl/src/osal/lv_sdl2.su ./Display_Driver/lvgl/src/osal/lv_windows.cyclo ./Display_Driver/lvgl/src/osal/lv_windows.d ./Display_Driver/lvgl/src/osal/lv_windows.o ./Display_Driver/lvgl/src/osal/lv_windows.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-osal

