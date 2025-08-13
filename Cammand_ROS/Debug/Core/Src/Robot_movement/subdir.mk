################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Robot_movement/Robot_base_movement.c 

OBJS += \
./Core/Src/Robot_movement/Robot_base_movement.o 

C_DEPS += \
./Core/Src/Robot_movement/Robot_base_movement.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Robot_movement/%.o Core/Src/Robot_movement/%.su Core/Src/Robot_movement/%.cyclo: ../Core/Src/Robot_movement/%.c Core/Src/Robot_movement/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Robot_movement

clean-Core-2f-Src-2f-Robot_movement:
	-$(RM) ./Core/Src/Robot_movement/Robot_base_movement.cyclo ./Core/Src/Robot_movement/Robot_base_movement.d ./Core/Src/Robot_movement/Robot_base_movement.o ./Core/Src/Robot_movement/Robot_base_movement.su

.PHONY: clean-Core-2f-Src-2f-Robot_movement

