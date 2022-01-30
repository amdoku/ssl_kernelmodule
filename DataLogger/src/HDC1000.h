/*
 * HDC1000.h
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#ifndef HDC1000_H_
#define HDC1000_H_

#include "KernelDevice.h"

#include <string>


class HDC1000 final : public KernelDevice {

public:
	HDC1000();

	//
	// Returns temperature and relative humidity if possible
	// If the bool return value is true, the read operation has been successful.
	// It is false otherwise.
	// There are two
	bool Read(float &temperature, float &relativeHumidity);

};

#endif /* HDC1000_H_ */
