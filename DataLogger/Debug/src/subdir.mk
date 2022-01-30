################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/DataLogger.cpp \
../src/DeliveryReporterCallback.cpp \
../src/FPGAManager.cpp \
../src/HDC1000.cpp \
../src/InfluxDBMessageHelper.cpp \
../src/KafkaMessageProducer.cpp \
../src/KernelDevice.cpp \
../src/MPU9250.cpp 

OBJS += \
./src/DataLogger.o \
./src/DeliveryReporterCallback.o \
./src/FPGAManager.o \
./src/HDC1000.o \
./src/InfluxDBMessageHelper.o \
./src/KafkaMessageProducer.o \
./src/KernelDevice.o \
./src/MPU9250.o 

CPP_DEPS += \
./src/DataLogger.d \
./src/DeliveryReporterCallback.d \
./src/FPGAManager.d \
./src/HDC1000.d \
./src/InfluxDBMessageHelper.d \
./src/KafkaMessageProducer.d \
./src/KernelDevice.d \
./src/MPU9250.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-poky-linux-gnueabi-g++ $(LDFLAGS) $(LIBS) -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a9 --sysroot=$(PKG_CONFIG_SYSROOT_DIR) -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


