################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../icode/rs485/rs485.c 

OBJS += \
./icode/rs485/rs485.o 

C_DEPS += \
./icode/rs485/rs485.d 


# Each subdirectory must supply rules for building sources it contributes
icode/rs485/%.o icode/rs485/%.su icode/rs485/%.cyclo: ../icode/rs485/%.c icode/rs485/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-icode-2f-rs485

clean-icode-2f-rs485:
	-$(RM) ./icode/rs485/rs485.cyclo ./icode/rs485/rs485.d ./icode/rs485/rs485.o ./icode/rs485/rs485.su

.PHONY: clean-icode-2f-rs485

