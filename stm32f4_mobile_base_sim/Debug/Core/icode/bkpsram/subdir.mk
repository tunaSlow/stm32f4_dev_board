################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/icode/bkpsram/bkpsram.c 

OBJS += \
./Core/icode/bkpsram/bkpsram.o 

C_DEPS += \
./Core/icode/bkpsram/bkpsram.d 


# Each subdirectory must supply rules for building sources it contributes
Core/icode/bkpsram/%.o Core/icode/bkpsram/%.su Core/icode/bkpsram/%.cyclo: ../Core/icode/bkpsram/%.c Core/icode/bkpsram/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-icode-2f-bkpsram

clean-Core-2f-icode-2f-bkpsram:
	-$(RM) ./Core/icode/bkpsram/bkpsram.cyclo ./Core/icode/bkpsram/bkpsram.d ./Core/icode/bkpsram/bkpsram.o ./Core/icode/bkpsram/bkpsram.su

.PHONY: clean-Core-2f-icode-2f-bkpsram

