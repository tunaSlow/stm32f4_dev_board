################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.c \
../Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.c \
../Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.c \
../Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.c \
../Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.o \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.o \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.o \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.o \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.d \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.d \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.d \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.d \
./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/others/test/%.o Middlewares/Third_Party/lvgl/src/others/test/%.su Middlewares/Third_Party/lvgl/src/others/test/%.cyclo: ../Middlewares/Third_Party/lvgl/src/others/test/%.c Middlewares/Third_Party/lvgl/src/others/test/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App/lvgl_port" -I"/home/tuna/STM32CubeIDE/workspace_1.18.1/stm32f4_dev_board/f4_lcd_display/App" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-test

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-test:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.cyclo ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.d ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.o ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_display.su ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.cyclo ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.d ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.o ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_helpers.su ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.cyclo ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.d ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.o ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev.su ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.cyclo ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.d ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.o ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_indev_gesture.su ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.cyclo ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.d ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.o ./Middlewares/Third_Party/lvgl/src/others/test/lv_test_screenshot_compare.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-others-2f-test

