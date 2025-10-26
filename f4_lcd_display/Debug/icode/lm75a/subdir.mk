################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../icode/lm75a/lm75a.c 

OBJS += \
./icode/lm75a/lm75a.o 

C_DEPS += \
./icode/lm75a/lm75a.d 


# Each subdirectory must supply rules for building sources it contributes
icode/lm75a/%.o icode/lm75a/%.su icode/lm75a/%.cyclo: ../icode/lm75a/%.c icode/lm75a/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-icode-2f-lm75a

clean-icode-2f-lm75a:
	-$(RM) ./icode/lm75a/lm75a.cyclo ./icode/lm75a/lm75a.d ./icode/lm75a/lm75a.o ./icode/lm75a/lm75a.su

.PHONY: clean-icode-2f-lm75a

