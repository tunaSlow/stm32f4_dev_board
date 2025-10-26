################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/tim/tim.c 

OBJS += \
./Core/icode/tim/tim.o 

C_DEPS += \
./Core/icode/tim/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/tim/%.o Core/icode/tim/%.su Core/icode/tim/%.cyclo: ../Core/icode/tim/%.c Core/icode/tim/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../USB_HOST/App -I../USB_HOST/Target -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-tim

clean-Core-2f-icode-2f-tim:
	-$(RM) ./Core/icode/tim/tim.cyclo ./Core/icode/tim/tim.d ./Core/icode/tim/tim.o ./Core/icode/tim/tim.su

.PHONY: clean-Core-2f-icode-2f-tim

