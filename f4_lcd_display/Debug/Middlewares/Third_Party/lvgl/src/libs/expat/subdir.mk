################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.c \
../Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.c \
../Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.c \
../Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.c \
../Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.c 

OBJS += \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.o \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.o \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.o \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.o \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.o 

C_DEPS += \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.d \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.d \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.d \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.d \
./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/lvgl/src/libs/expat/%.o Middlewares/Third_Party/lvgl/src/libs/expat/%.su Middlewares/Third_Party/lvgl/src/libs/expat/%.cyclo: ../Middlewares/Third_Party/lvgl/src/libs/expat/%.c Middlewares/Third_Party/lvgl/src/libs/expat/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG '-DLV_CONF_PATH="lv_conf.h"' -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Inc" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Middlewares/Third_Party/lvgl/src" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/App" -I"C:/Users/user/Documents/GitHub/stm32f4_dev_board/f4_lcd_display/Core/Src/ui" -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-expat

clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-expat:
	-$(RM) ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.cyclo ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.d ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.o ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlparse.su ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.cyclo ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.d ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.o ./Middlewares/Third_Party/lvgl/src/libs/expat/xmlrole.su ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.cyclo ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.d ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.o ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok.su ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.cyclo ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.d ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.o ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_impl.su ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.cyclo ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.d ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.o ./Middlewares/Third_Party/lvgl/src/libs/expat/xmltok_ns.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-lvgl-2f-src-2f-libs-2f-expat

