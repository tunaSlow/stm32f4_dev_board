################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.c \
../Display_Driver/lvgl/tests/src/test_cases_perf/test_label.c \
../Display_Driver/lvgl/tests/src/test_cases_perf/test_math.c 

OBJS += \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.o \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.o \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.o 

C_DEPS += \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.d \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.d \
./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/tests/src/test_cases_perf/%.o Display_Driver/lvgl/tests/src/test_cases_perf/%.su Display_Driver/lvgl/tests/src/test_cases_perf/%.cyclo: ../Display_Driver/lvgl/tests/src/test_cases_perf/%.c Display_Driver/lvgl/tests/src/test_cases_perf/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf

clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf:
	-$(RM) ./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.cyclo ./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.d ./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.o ./Display_Driver/lvgl/tests/src/test_cases_perf/test_chart.su ./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.cyclo ./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.d ./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.o ./Display_Driver/lvgl/tests/src/test_cases_perf/test_label.su ./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.cyclo ./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.d ./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.o ./Display_Driver/lvgl/tests/src/test_cases_perf/test_math.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-tests-2f-src-2f-test_cases_perf

