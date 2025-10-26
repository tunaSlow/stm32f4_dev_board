################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/rs485/rs485.c 

OBJS += \
./Core/icode/rs485/rs485.o 

C_DEPS += \
./Core/icode/rs485/rs485.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/rs485/%.o Core/icode/rs485/%.su Core/icode/rs485/%.cyclo: ../Core/icode/rs485/%.c Core/icode/rs485/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-rs485

clean-Core-2f-icode-2f-rs485:
	-$(RM) ./Core/icode/rs485/rs485.cyclo ./Core/icode/rs485/rs485.d ./Core/icode/rs485/rs485.o ./Core/icode/rs485/rs485.su

.PHONY: clean-Core-2f-icode-2f-rs485

