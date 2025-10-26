################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/lcd/lcd.c \
../Core/icode/lcd/touch.c 

OBJS += \
./Core/icode/lcd/lcd.o \
./Core/icode/lcd/touch.o 

C_DEPS += \
./Core/icode/lcd/lcd.d \
./Core/icode/lcd/touch.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/lcd/%.o Core/icode/lcd/%.su Core/icode/lcd/%.cyclo: ../Core/icode/lcd/%.c Core/icode/lcd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-lcd

clean-Core-2f-icode-2f-lcd:
	-$(RM) ./Core/icode/lcd/lcd.cyclo ./Core/icode/lcd/lcd.d ./Core/icode/lcd/lcd.o ./Core/icode/lcd/lcd.su ./Core/icode/lcd/touch.cyclo ./Core/icode/lcd/touch.d ./Core/icode/lcd/touch.o ./Core/icode/lcd/touch.su

.PHONY: clean-Core-2f-icode-2f-lcd

