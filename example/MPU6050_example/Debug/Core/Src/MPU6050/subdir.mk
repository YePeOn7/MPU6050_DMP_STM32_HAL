################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/MPU6050/IOI2C.c \
../Core/Src/MPU6050/MPU6050.c \
../Core/Src/MPU6050/inv_mpu.c \
../Core/Src/MPU6050/inv_mpu_dmp_motion_driver.c 

C_DEPS += \
./Core/Src/MPU6050/IOI2C.d \
./Core/Src/MPU6050/MPU6050.d \
./Core/Src/MPU6050/inv_mpu.d \
./Core/Src/MPU6050/inv_mpu_dmp_motion_driver.d 

OBJS += \
./Core/Src/MPU6050/IOI2C.o \
./Core/Src/MPU6050/MPU6050.o \
./Core/Src/MPU6050/inv_mpu.o \
./Core/Src/MPU6050/inv_mpu_dmp_motion_driver.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/MPU6050/%.o Core/Src/MPU6050/%.su: ../Core/Src/MPU6050/%.c Core/Src/MPU6050/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/yp7/dev/MPU6050_DMP_STM32_HAL/example/MPU6050_example/Core/Inc/MPU6050" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-MPU6050

clean-Core-2f-Src-2f-MPU6050:
	-$(RM) ./Core/Src/MPU6050/IOI2C.d ./Core/Src/MPU6050/IOI2C.o ./Core/Src/MPU6050/IOI2C.su ./Core/Src/MPU6050/MPU6050.d ./Core/Src/MPU6050/MPU6050.o ./Core/Src/MPU6050/MPU6050.su ./Core/Src/MPU6050/inv_mpu.d ./Core/Src/MPU6050/inv_mpu.o ./Core/Src/MPU6050/inv_mpu.su ./Core/Src/MPU6050/inv_mpu_dmp_motion_driver.d ./Core/Src/MPU6050/inv_mpu_dmp_motion_driver.o ./Core/Src/MPU6050/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Core-2f-Src-2f-MPU6050

