################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_freertos.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_linux.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_mqx.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_os.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_os_none.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_pthread.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.c \
../Middlewares/Third_Party/lvgl/src/osal/lv_windows.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_linux.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_os.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.o \
./Middlewares/Third_Party/lvgl/src/osal/lv_windows.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_linux.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_os.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.d \
./Middlewares/Third_Party/lvgl/src/osal/lv_windows.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/osal/%.o Middlewares/Third_Party/lvgl/src/osal/%.su Middlewares/Third_Party/lvgl/src/osal/%.cyclo: ../Middlewares/Third_Party/lvgl/src/osal/%.c Middlewares/Third_Party/lvgl/src/osal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/App" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-osal

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-osal:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.d ./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.o ./Middlewares/Third_Party/lvgl/src/osal/lv_cmsis_rtos2.su ./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.d ./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.o ./Middlewares/Third_Party/lvgl/src/osal/lv_freertos.su ./Middlewares/Third_Party/lvgl/src/osal/lv_linux.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_linux.d ./Middlewares/Third_Party/lvgl/src/osal/lv_linux.o ./Middlewares/Third_Party/lvgl/src/osal/lv_linux.su ./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.d ./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.o ./Middlewares/Third_Party/lvgl/src/osal/lv_mqx.su ./Middlewares/Third_Party/lvgl/src/osal/lv_os.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_os.d ./Middlewares/Third_Party/lvgl/src/osal/lv_os.o ./Middlewares/Third_Party/lvgl/src/osal/lv_os.su ./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.d ./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.o ./Middlewares/Third_Party/lvgl/src/osal/lv_os_none.su ./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.d ./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.o ./Middlewares/Third_Party/lvgl/src/osal/lv_pthread.su ./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.d ./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.o ./Middlewares/Third_Party/lvgl/src/osal/lv_rtthread.su ./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.d ./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.o ./Middlewares/Third_Party/lvgl/src/osal/lv_sdl2.su ./Middlewares/Third_Party/lvgl/src/osal/lv_windows.cyclo ./Middlewares/Third_Party/lvgl/src/osal/lv_windows.d ./Middlewares/Third_Party/lvgl/src/osal/lv_windows.o ./Middlewares/Third_Party/lvgl/src/osal/lv_windows.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-osal

