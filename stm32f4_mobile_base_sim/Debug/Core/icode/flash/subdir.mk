################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/flash/flash.c 

OBJS += \
./Core/icode/flash/flash.o 

C_DEPS += \
./Core/icode/flash/flash.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/flash/%.o Core/icode/flash/%.su Core/icode/flash/%.cyclo: ../Core/icode/flash/%.c Core/icode/flash/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-flash

clean-Core-2f-icode-2f-flash:
	-$(RM) ./Core/icode/flash/flash.cyclo ./Core/icode/flash/flash.d ./Core/icode/flash/flash.o ./Core/icode/flash/flash.su

.PHONY: clean-Core-2f-icode-2f-flash

