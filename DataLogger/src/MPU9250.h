/*
 * MPU9250Impl.h
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include "KernelDevice.h"

#include <signal.h>
#include <vector>

struct MPUAxis {
	float x;
	float y;
	float z;
};

struct MPUData {
	int64_t timestamp_ns;
	MPUAxis accel;
	MPUAxis gyro;
	MPUAxis mag;
};

class MPU9250 final : private KernelDevice {
public:
	MPU9250();

	bool RegisterInterrupt(void (*handler)(int));

	bool Read(MPUData& rMPU);

	bool ReadInterruptBuffer(std::vector<MPUData>& rMPUVec);

private:
	float accelSensitivity, gyroSensitivity, magSensitivityInv;

	inline float calcAccel(int16_t value) {
		return static_cast<float>(value) / accelSensitivity;
	}

	inline float calcGyro(int16_t value) {
		return static_cast<float>(value) / gyroSensitivity;
	}

	inline float calcMag(int16_t value) {
		return static_cast<float>(value) * magSensitivityInv;
	}

	struct sigaction signalHandler;
};

#endif /* MPU9250_H_ */
