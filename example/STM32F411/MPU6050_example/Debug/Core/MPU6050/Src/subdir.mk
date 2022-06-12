################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/MPU6050/Src/IOI2C.c \
../Core/MPU6050/Src/MPU6050.c \
../Core/MPU6050/Src/inv_mpu.c \
../Core/MPU6050/Src/inv_mpu_dmp_motion_driver.c 

C_DEPS += \
./Core/MPU6050/Src/IOI2C.d \
./Core/MPU6050/Src/MPU6050.d \
./Core/MPU6050/Src/inv_mpu.d \
./Core/MPU6050/Src/inv_mpu_dmp_motion_driver.d 

OBJS += \
./Core/MPU6050/Src/IOI2C.o \
./Core/MPU6050/Src/MPU6050.o \
./Core/MPU6050/Src/inv_mpu.o \
./Core/MPU6050/Src/inv_mpu_dmp_motion_driver.o 


# Each subdirectory must supply rules for building sources it contributes
Core/MPU6050/Src/%.o Core/MPU6050/Src/%.su: ../Core/MPU6050/Src/%.c Core/MPU6050/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I"/home/yp7/dev/ROS_based_Robot/STM32_ws/MPU6050_DMP_STM32_HAL/example/STM32F411/MPU6050_example/Core/MPU6050/Inc" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-MPU6050-2f-Src

clean-Core-2f-MPU6050-2f-Src:
	-$(RM) ./Core/MPU6050/Src/IOI2C.d ./Core/MPU6050/Src/IOI2C.o ./Core/MPU6050/Src/IOI2C.su ./Core/MPU6050/Src/MPU6050.d ./Core/MPU6050/Src/MPU6050.o ./Core/MPU6050/Src/MPU6050.su ./Core/MPU6050/Src/inv_mpu.d ./Core/MPU6050/Src/inv_mpu.o ./Core/MPU6050/Src/inv_mpu.su ./Core/MPU6050/Src/inv_mpu_dmp_motion_driver.d ./Core/MPU6050/Src/inv_mpu_dmp_motion_driver.o ./Core/MPU6050/Src/inv_mpu_dmp_motion_driver.su

.PHONY: clean-Core-2f-MPU6050-2f-Src

