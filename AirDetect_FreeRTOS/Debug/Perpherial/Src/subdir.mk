################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Perpherial/Src/bluetooth.c \
../Perpherial/Src/bmp280.c \
../Perpherial/Src/dht11.c \
../Perpherial/Src/gy30.c \
../Perpherial/Src/lcd1602.c 

OBJS += \
./Perpherial/Src/bluetooth.o \
./Perpherial/Src/bmp280.o \
./Perpherial/Src/dht11.o \
./Perpherial/Src/gy30.o \
./Perpherial/Src/lcd1602.o 

C_DEPS += \
./Perpherial/Src/bluetooth.d \
./Perpherial/Src/bmp280.d \
./Perpherial/Src/dht11.d \
./Perpherial/Src/gy30.d \
./Perpherial/Src/lcd1602.d 


# Each subdirectory must supply rules for building sources it contributes
Perpherial/Src/bluetooth.o: ../Perpherial/Src/bluetooth.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"E:/MyCode/AirDetect_FreeRTOS/Perpherial/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Perpherial/Src/bluetooth.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Perpherial/Src/bmp280.o: ../Perpherial/Src/bmp280.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"E:/MyCode/AirDetect_FreeRTOS/Perpherial/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Perpherial/Src/bmp280.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Perpherial/Src/dht11.o: ../Perpherial/Src/dht11.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"E:/MyCode/AirDetect_FreeRTOS/Perpherial/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Perpherial/Src/dht11.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Perpherial/Src/gy30.o: ../Perpherial/Src/gy30.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"E:/MyCode/AirDetect_FreeRTOS/Perpherial/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Perpherial/Src/gy30.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Perpherial/Src/lcd1602.o: ../Perpherial/Src/lcd1602.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32F103xB -DDEBUG -c -I../Core/Inc -I"E:/MyCode/AirDetect_FreeRTOS/Perpherial/Inc" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Perpherial/Src/lcd1602.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

