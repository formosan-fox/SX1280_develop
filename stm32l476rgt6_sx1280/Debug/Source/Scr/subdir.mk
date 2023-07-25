################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
C:/Users/frank/Desktop/sx1280/Source/Scr/sx1280.cpp 

OBJS += \
./Source/Scr/sx1280.o 

CPP_DEPS += \
./Source/Scr/sx1280.d 


# Each subdirectory must supply rules for building sources it contributes
Source/Scr/sx1280.o: C:/Users/frank/Desktop/sx1280/Source/Scr/sx1280.cpp Source/Scr/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../../Source/Inc -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Source-2f-Scr

clean-Source-2f-Scr:
	-$(RM) ./Source/Scr/sx1280.cyclo ./Source/Scr/sx1280.d ./Source/Scr/sx1280.o ./Source/Scr/sx1280.su

.PHONY: clean-Source-2f-Scr

