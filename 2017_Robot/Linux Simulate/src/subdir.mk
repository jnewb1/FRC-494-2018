################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/ADIS16448_IMU.cpp \
../src/Aton.cpp \
../src/Base.cpp \
../src/Cal.cpp \
../src/Driving.cpp \
../src/JoyStick.cpp \
../src/Motion.cpp \
../src/Motor.cpp \
../src/Robot.cpp \
../src/Timer.cpp \
../src/UserCode.cpp \
../src/iEncoder.cpp \
../src/jpegdecodertest.cpp 

OBJS += \
./src/ADIS16448_IMU.o \
./src/Aton.o \
./src/Base.o \
./src/Cal.o \
./src/Driving.o \
./src/JoyStick.o \
./src/Motion.o \
./src/Motor.o \
./src/Robot.o \
./src/Timer.o \
./src/UserCode.o \
./src/iEncoder.o \
./src/jpegdecodertest.o 

CPP_DEPS += \
./src/ADIS16448_IMU.d \
./src/Aton.d \
./src/Base.d \
./src/Cal.d \
./src/Driving.d \
./src/JoyStick.d \
./src/Motion.d \
./src/Motor.d \
./src/Robot.d \
./src/Timer.d \
./src/UserCode.d \
./src/iEncoder.d \
./src/jpegdecodertest.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -I"C:\Users\robotics/wpilib/cpp/current/include" -I"C:\Users\robotics\Desktop\CPP_Mecanum_CANTalon_Example\src" -I"C:\Users\robotics/wpilib/user/cpp/include" -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


