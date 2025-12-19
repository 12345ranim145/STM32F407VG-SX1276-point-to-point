################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Drivers/SX1276/SX1276.c \
../Core/Drivers/SX1276/SX1276_hw.c 

OBJS += \
./Core/Drivers/SX1276/SX1276.o \
./Core/Drivers/SX1276/SX1276_hw.o 

C_DEPS += \
./Core/Drivers/SX1276/SX1276.d \
./Core/Drivers/SX1276/SX1276_hw.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Drivers/SX1276/%.o Core/Drivers/SX1276/%.su Core/Drivers/SX1276/%.cyclo: ../Core/Drivers/SX1276/%.c Core/Drivers/SX1276/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/ranim/Downloads/SX1276-master/SX1278-master/driver" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Drivers-2f-SX1276

clean-Core-2f-Drivers-2f-SX1276:
	-$(RM) ./Core/Drivers/SX1276/SX1276.cyclo ./Core/Drivers/SX1276/SX1276.d ./Core/Drivers/SX1276/SX1276.o ./Core/Drivers/SX1276/SX1276.su ./Core/Drivers/SX1276/SX1276_hw.cyclo ./Core/Drivers/SX1276/SX1276_hw.d ./Core/Drivers/SX1276/SX1276_hw.o ./Core/Drivers/SX1276/SX1276_hw.su

.PHONY: clean-Core-2f-Drivers-2f-SX1276

