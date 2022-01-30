/*
 * HDC1000.cpp
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#include "HDC1000.h"

static const std::string DRIVER_FILE_NAME = "hdc1000";

HDC1000::HDC1000(): KernelDevice(DRIVER_FILE_NAME) {

}


bool HDC1000::Read(float &temperature, float &relativeHumidity) {
	temperature = 0.0f;
	relativeHumidity = 0.0f;

	uint32_t data = 0;
	
#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
	bool result = true;
	temperature = 21.0;
	relativeHumidity = 40;
#else
	bool result = ReadFromDevice(&data, sizeof(data));

	if (result) {

		uint16_t temp = static_cast<uint16_t>(data >> 16);
		uint16_t hum = static_cast<uint16_t>(data >> 0);

		temperature = (static_cast<float>(temp) / 65536.0f) * 165 - 40;
		relativeHumidity = (static_cast<float>(hum) / 65536.0f) * 100;

	}
#endif

	return result;

	//temperature = 29;
	//relativeHumidity = 15;
	//return true;
}
