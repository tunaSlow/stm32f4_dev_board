################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../icode/aliyun/esp8266/esp8266.c 

OBJS += \
./icode/aliyun/esp8266/esp8266.o 

C_DEPS += \
./icode/aliyun/esp8266/esp8266.d 


# Each subdirectory must supply rules for building sources it contributes
icode/aliyun/esp8266/%.o: ../icode/aliyun/esp8266/%.c icode/aliyun/esp8266/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-icode-2f-aliyun-2f-esp8266

clean-icode-2f-aliyun-2f-esp8266:
	-$(RM) ./icode/aliyun/esp8266/esp8266.d ./icode/aliyun/esp8266/esp8266.o

.PHONY: clean-icode-2f-aliyun-2f-esp8266

