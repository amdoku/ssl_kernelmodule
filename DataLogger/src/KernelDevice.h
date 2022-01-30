/*
 * KernelDevice.h
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */

#ifndef KERNELDEVICE_H_
#define KERNELDEVICE_H_

#include <string>
#include <mutex>
#include <shared_mutex>
#include <functional>

#include "IFPGAEvent.h"

class KernelDevice: private IFPGAEvent {
public:
	KernelDevice(std::string pathToDevice);
	virtual ~KernelDevice();

	//
	// read data from kernel module
	bool ReadFromDevice(void* data, size_t length);

	//
	// read module parameter
	bool ReadModuleParameter(const std::string paramName, std::string& output);


protected:
	void LockedAccess(std::function<void(int fileID)> f);

private:
	const std::string pathToDevice;
	const std::string pathToModuleParameters;

	int kernelModuleFile;
	std::mutex mutex;

	virtual void onFPGARelease() override;
	virtual void onFPGAAcquire() override;

	void _unsafeAcquire();
	void _unsafeRelease();

	void _loadIfNotOpen();

};

#endif /* KERNELDEVICE_H_ */
