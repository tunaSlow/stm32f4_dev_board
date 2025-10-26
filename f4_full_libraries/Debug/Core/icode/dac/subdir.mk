################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/dac/dac.c 

OBJS += \
./Core/icode/dac/dac.o 

C_DEPS += \
./Core/icode/dac/dac.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/dac/%.o Core/icode/dac/%.su Core/icode/dac/%.cyclo: ../Core/icode/dac/%.c Core/icode/dac/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-dac

clean-Core-2f-icode-2f-dac:
	-$(RM) ./Core/icode/dac/dac.cyclo ./Core/icode/dac/dac.d ./Core/icode/dac/dac.o ./Core/icode/dac/dac.su

.PHONY: clean-Core-2f-icode-2f-dac

