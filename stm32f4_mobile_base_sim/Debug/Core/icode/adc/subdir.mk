################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/adc/adc.c 

OBJS += \
./Core/icode/adc/adc.o 

C_DEPS += \
./Core/icode/adc/adc.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/adc/%.o Core/icode/adc/%.su Core/icode/adc/%.cyclo: ../Core/icode/adc/%.c Core/icode/adc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-adc

clean-Core-2f-icode-2f-adc:
	-$(RM) ./Core/icode/adc/adc.cyclo ./Core/icode/adc/adc.d ./Core/icode/adc/adc.o ./Core/icode/adc/adc.su

.PHONY: clean-Core-2f-icode-2f-adc

