################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/lib/sm_bypass.c 

OBJS += \
./Core/Src/lib/sm_bypass.o 

C_DEPS += \
./Core/Src/lib/sm_bypass.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/lib/%.o Core/Src/lib/%.su: ../Core/Src/lib/%.c Core/Src/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303xE -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-lib

clean-Core-2f-Src-2f-lib:
	-$(RM) ./Core/Src/lib/sm_bypass.d ./Core/Src/lib/sm_bypass.o ./Core/Src/lib/sm_bypass.su

.PHONY: clean-Core-2f-Src-2f-lib

