################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Bracket_track/Track.c 

OBJS += \
./Core/Src/Bracket_track/Track.o 

C_DEPS += \
./Core/Src/Bracket_track/Track.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Bracket_track/%.o Core/Src/Bracket_track/%.su Core/Src/Bracket_track/%.cyclo: ../Core/Src/Bracket_track/%.c Core/Src/Bracket_track/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Bracket_track

clean-Core-2f-Src-2f-Bracket_track:
	-$(RM) ./Core/Src/Bracket_track/Track.cyclo ./Core/Src/Bracket_track/Track.d ./Core/Src/Bracket_track/Track.o ./Core/Src/Bracket_track/Track.su

.PHONY: clean-Core-2f-Src-2f-Bracket_track

