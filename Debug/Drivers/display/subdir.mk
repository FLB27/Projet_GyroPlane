################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/display/max7219_Yncrea2.c 

OBJS += \
./Drivers/display/max7219_Yncrea2.o 

C_DEPS += \
./Drivers/display/max7219_Yncrea2.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/display/%.o Drivers/display/%.su Drivers/display/%.cyclo: ../Drivers/display/%.c Drivers/display/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L152xE -c -I../Core/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc -I../Drivers/STM32L1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L1xx/Include -I../Drivers/CMSIS/Include -I../X-CUBE-MEMS1/Target -I../Drivers/BSP/Components/lsm6dso -I../Drivers/BSP/Components/lis2dw12 -I../Drivers/BSP/Components/lis2mdl -I../Drivers/BSP/Components/hts221 -I../Drivers/BSP/Components/lps22hh -I../Drivers/BSP/Components/stts751 -I../Drivers/BSP/IKS01A3 -I../Drivers/BSP/Components/Common -I"C:/Users/falla/OneDrive/Bureau/µP/STM32/Projet STM32/Projet_GyroPlane/Drivers/display" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-display

clean-Drivers-2f-display:
	-$(RM) ./Drivers/display/max7219_Yncrea2.cyclo ./Drivers/display/max7219_Yncrea2.d ./Drivers/display/max7219_Yncrea2.o ./Drivers/display/max7219_Yncrea2.su

.PHONY: clean-Drivers-2f-display

