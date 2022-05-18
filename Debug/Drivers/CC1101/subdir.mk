################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CC1101/CC1101.c \
../Drivers/CC1101/CC1101_port.c 

OBJS += \
./Drivers/CC1101/CC1101.o \
./Drivers/CC1101/CC1101_port.o 

C_DEPS += \
./Drivers/CC1101/CC1101.d \
./Drivers/CC1101/CC1101_port.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CC1101/CC1101.o: ../Drivers/CC1101/CC1101.c Drivers/CC1101/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/workspace/CC1101_HAL_STM32/Drivers/CC1101" -I"C:/ST/workspace/CC1101_HAL_STM32/Core/Src" -I"C:/ST/workspace/CC1101_HAL_STM32/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CC1101/CC1101.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/CC1101/CC1101_port.o: ../Drivers/CC1101/CC1101_port.c Drivers/CC1101/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F042x6 -c -I../Core/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc -I../Drivers/STM32F0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F0xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/workspace/CC1101_HAL_STM32/Drivers/CC1101" -I"C:/ST/workspace/CC1101_HAL_STM32/Core/Src" -I"C:/ST/workspace/CC1101_HAL_STM32/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CC1101/CC1101_port.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

