################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/util/delay/delay.c 

OBJS += \
./Core/util/delay/delay.o 

C_DEPS += \
./Core/util/delay/delay.d 


# Each subdirectory must supply rules for building sources it contributes
Core/util/delay/%.o Core/util/delay/%.su Core/util/delay/%.cyclo: ../Core/util/delay/%.c Core/util/delay/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-util-2f-delay

clean-Core-2f-util-2f-delay:
	-$(RM) ./Core/util/delay/delay.cyclo ./Core/util/delay/delay.d ./Core/util/delay/delay.o ./Core/util/delay/delay.su

.PHONY: clean-Core-2f-util-2f-delay

