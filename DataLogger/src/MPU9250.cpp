/*
 * MPU9250Impl.cpp
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */

#include "MPU9250.h"

#include "mpu9250_def.h"

#include <stdexcept>
#include <iostream>
#include <cassert>

// to get the current pid
#include <sys/ioctl.h>

#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
#include "TimeHelper.h"
#endif

static const std::string DRIVER_FILE_NAME = "mpu9250";
static const std::string ACCL_DIV = "ACCL_DIV";
static const std::string GYRO_DIV = "GYRO_DIV";
static const std::string MAGN_MUL = "MAGN_MUL";

//
// inverse of:
// https://godbolt.org/#g:!((g:!((g:!((h:codeEditor,i:(filename:'1',fontScale:14,fontUsePx:'0',j:1,lang:c%2B%2B,selection:(endColumn:33,endLineNumber:12,positionColumn:33,positionLineNumber:12,selectionStartColumn:33,selectionStartLineNumber:12,startColumn:33,startLineNumber:12),source:'%23include+%3Ciostream%3E%0A%0Anamespace+%7B%0A++++float+const+divVal+%3D+16.4f%3B%0A++++float+const+mpuMul+%3D+0.6f%3B%0A%7D%0A%0Aint+main()+%7B%0A++++auto+ptr+%3D+reinterpret_cast%3Cint32_t+const*%3E(%26divVal)%3B%0A++++std::printf(%220x%25x%5Cn%22,+*ptr)%3B%0A++++ptr+%3D+reinterpret_cast%3Cint32_t+const*%3E(%26mpuMul)%3B%0A++++std::printf(%220x%25x%5Cn%22,+*ptr)%3B%0A++++return+0%3B%0A%7D%0A'),l:'5',n:'0',o:'C%2B%2B+source+%231',t:'0')),k:33.333333333333336,l:'4',n:'0',o:'',s:0,t:'0'),(g:!((h:compiler,i:(compiler:g112,filters:(b:'0',binary:'1',commentOnly:'0',demangle:'0',directives:'0',execute:'0',intel:'0',libraryCode:'0',trim:'1'),flagsViewOpen:'1',fontScale:14,fontUsePx:'0',j:1,lang:c%2B%2B,libs:!(),options:'-O3',selection:(endColumn:1,endLineNumber:1,positionColumn:1,positionLineNumber:1,selectionStartColumn:1,selectionStartLineNumber:1,startColumn:1,startLineNumber:1),source:1,tree:'1'),l:'5',n:'0',o:'x86-64+gcc+11.2+(C%2B%2B,+Editor+%231,+Compiler+%231)',t:'0')),k:33.333333333333336,l:'4',n:'0',o:'',s:0,t:'0'),(g:!((h:output,i:(compiler:1,editor:1,fontScale:14,fontUsePx:'0',tree:'1',wrap:'1'),l:'5',n:'0',o:'Output+of+x86-64+gcc+11.2+(Compiler+%231)',t:'0')),k:33.33333333333333,l:'4',n:'0',o:'',s:0,t:'0')),l:'2',n:'0',o:'',t:'0')),version:4
inline float reinterpretIntToFloat(int i) {
	float* toFloat = reinterpret_cast<float*>(&i);
	return *toFloat;
}

MPU9250::MPU9250() :
		KernelDevice(DRIVER_FILE_NAME) {

#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
	accelSensitivity = 1;
	gyroSensitivity = 1;
	magSensitivityInv = 1;
#else
	std::string tmp;
	bool result = ReadModuleParameter(ACCL_DIV, tmp);
	if (result) {
		accelSensitivity = std::stof(tmp);
	} else {
		throw std::runtime_error("Unable to read ACCL_DIV from mpu module!");
	}

	assert(sizeof(float) == sizeof(int));

	result = ReadModuleParameter(GYRO_DIV, tmp);
	if (result) {
		int i = std::stoi(tmp);
		gyroSensitivity = reinterpretIntToFloat(i);
	} else {
		throw std::runtime_error("Unable to read GYRO_DIV from mpu module!");
	}

	result = ReadModuleParameter(MAGN_MUL, tmp);
	if (result) {
		int i = std::stoi(tmp);
		magSensitivityInv = reinterpretIntToFloat(i);
	} else {
		throw std::runtime_error("Unable to read MAGN_MUL from mpu module!");
	}
#endif


	std::cout << "MPU-Params:" << std::endl;
	std::cout << ACCL_DIV << ": " << accelSensitivity << std::endl;
	std::cout << GYRO_DIV << ": " << gyroSensitivity << std::endl;
	std::cout << MAGN_MUL << ": " << magSensitivityInv << std::endl;
	std::cout << std::endl;
}

bool MPU9250::Read(MPUData &rMPU) {

	mpu9250::mpu9250_t data;

#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
	bool status = true;
#else
	bool status = ReadFromDevice(&data, sizeof(mpu9250::mpu9250_t));
#endif

#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
	bool status = true;

	rMPU.accel.x = 0.1;
	rMPU.accel.y = 1;
	rMPU.accel.z = 0.5;

	rMPU.gyro.x = 0;
	rMPU.gyro.y = -1;
	rMPU.gyro.z = 0;

	rMPU.mag.x = 0.2;
	rMPU.mag.y = 0.3;
	rMPU.mag.z = 0.1;

	rMPU.timestamp_ns = getCurrentTime();
#else
	rMPU.accel.x = calcAccel(data.data.accel.x);
	rMPU.accel.y = calcAccel(data.data.accel.y);
	rMPU.accel.z = calcAccel(data.data.accel.z);

	rMPU.gyro.x = calcGyro(data.data.gyro.x);
	rMPU.gyro.y = calcGyro(data.data.gyro.y);
	rMPU.gyro.z = calcGyro(data.data.gyro.z);

	rMPU.mag.x = calcMag(data.data.mag.x);
	rMPU.mag.y = calcMag(data.data.mag.y);
	rMPU.mag.z = calcMag(data.data.mag.z);

	rMPU.timestamp_ns = data.timestamp_ns;
#endif

	return status;
}


bool MPU9250::ReadInterruptBuffer(std::vector<MPUData>& rMPUVec) {
	mpu9250::bumpData_t data;
	bool state = false;

	this->LockedAccess([&](int id) {
		if(ioctl(id, IOCTL_MPU9250_GIB_DATA, &data) == 0) {
			std::cout << "Reading interrupt buffer was successful!" << std::endl;
			state = true;
		} else {
			//std::cout << "Failed reading interrupt buffer" << std::endl;
		}

	});

	if(state) {
		rMPUVec.resize(IOCTL_MPU9250_GIB_DATA_SIZE * IOCTL_MPU9250_GIB_DATA_BLOCK_SIZE);

		int64_t startTime = data.timestamp_ns;

		for(size_t i = 0; i < IOCTL_MPU9250_GIB_DATA_BLOCK_SIZE; i++) {
			for(size_t j = 0; j < IOCTL_MPU9250_GIB_DATA_SIZE; j++) {
				mpu9250::mpu9250_data_t& meas = data.data[i][j].data;
				MPUData& rMPU = rMPUVec[i];

				rMPU.accel.x = calcAccel(meas.accel.x);
				rMPU.accel.y = calcAccel(meas.accel.y);
				rMPU.accel.z = calcAccel(meas.accel.z);

				rMPU.gyro.x = calcGyro(meas.gyro.x);
				rMPU.gyro.y = calcGyro(meas.gyro.y);
				rMPU.gyro.z = calcGyro(meas.gyro.z);

				rMPU.mag.x = calcMag(meas.mag.x);
				rMPU.mag.y = calcMag(meas.mag.y);
				rMPU.mag.z = calcMag(meas.mag.z);

				rMPU.timestamp_ns = startTime + (i*930136633439834133u) + j;
			}
		}

	} else {
		std::cout << "Failed reading interrupt buffer" << std::endl;
	}

	return state;
}

bool MPU9250::RegisterInterrupt(void (*handler)(int)) {
	bool state = false;

	this->LockedAccess([&state](int id) {
		if(ioctl(id, IOCTL_MPU9250_REGISTER_PID, SIGNAL_MPU9250) == 0) {
			std::cout << "Registering interrupt for signal: " << SIGNAL_MPU9250 << std::endl;
			state = true;
		} else {
			std::cout << "Error while registering interrupt for signal: " << SIGNAL_MPU9250 << std::endl;
		}

	});

	if(state) {
		signalHandler.sa_handler = handler;
		sigemptyset(&signalHandler.sa_mask);
		signalHandler.sa_flags = 0;

		// set new function for sighandler
		sigaction(SIGNAL_MPU9250, &signalHandler, nullptr);
	} else {
		std::cout << "Setting up signal interrupt failed" << std::endl;
	}

	return state;
}

