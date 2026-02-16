################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Display_Driver/lvgl/src/widgets/calendar/lv_calendar.c \
../Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.c \
../Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.c \
../Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.c 

OBJS += \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.o \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.o \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.o \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.o 

C_DEPS += \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.d \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.d \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.d \
./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.d 


# Each subdirectory must supply rules for building sources it contributes
Display_Driver/lvgl/src/widgets/calendar/%.o Display_Driver/lvgl/src/widgets/calendar/%.su Display_Driver/lvgl/src/widgets/calendar/%.cyclo: ../Display_Driver/lvgl/src/widgets/calendar/%.c Display_Driver/lvgl/src/widgets/calendar/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -I"/home/tuna/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Display_Driver/lvgl/src" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-calendar

clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-calendar:
	-$(RM) ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.cyclo ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.d ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.o ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar.su ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.cyclo ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.d ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.o ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_chinese.su ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.cyclo ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.d ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.o ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_arrow.su ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.cyclo ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.d ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.o ./Display_Driver/lvgl/src/widgets/calendar/lv_calendar_header_dropdown.su

.PHONY: clean-Display_Driver-2f-lvgl-2f-src-2f-widgets-2f-calendar

