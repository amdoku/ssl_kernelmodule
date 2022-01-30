/*
 * KernelDevice.cpp
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

#include <sys/ioctl.h>

#include "KernelDevice.h"
#include "FPGAManager.h"

static const std::string SYSTEM_DEVICE_PATH = "/dev/";
static const std::string SYSTEM_MODULE_PATH = "/sys/module/";
static const std::string SYSTEM_MODULE_PARAMETERS = "/parameters/";

KernelDevice::KernelDevice(const std::string pathToDevice) :
		pathToDevice(SYSTEM_DEVICE_PATH + pathToDevice),
		pathToModuleParameters(SYSTEM_MODULE_PATH + pathToDevice + SYSTEM_MODULE_PARAMETERS),
		kernelModuleFile(-1)
{
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	FPGAManager::registerEvent(this);
}

KernelDevice::~KernelDevice() {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const std::lock_guard<std::mutex> lock(mutex);

	// clean up ...

	FPGAManager::unregisterEvent(this);

	_unsafeRelease();
}

void KernelDevice::onFPGARelease() {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const std::lock_guard<std::mutex> lock(mutex);

	_unsafeRelease();

}

void KernelDevice::onFPGAAcquire() {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const std::lock_guard<std::mutex> lock(mutex);

	_unsafeAcquire();
}


void KernelDevice::_unsafeAcquire()
{
	if(kernelModuleFile < 0) {
		kernelModuleFile = open(pathToDevice.c_str(), O_RDONLY);
	}
}


void KernelDevice::_unsafeRelease()
{
	if(kernelModuleFile >= 0) {
		close(kernelModuleFile); kernelModuleFile = -1;
	}
}


void KernelDevice::_loadIfNotOpen() {
	if(kernelModuleFile < 0 && FPGAManager::IsFPGAAcquired()) {
		// try to init the device using the fpga callback
		// this will fail if the fpga is not aquired

		_unsafeAcquire();
	}
}


// read data from kernel module
bool KernelDevice::ReadFromDevice(void* data, size_t length) {
	// the read lock has to be acquired first because
	// there is a risk for a deadlock otherwise!
	const auto fpgaLock = FPGAManager::ScopedReadLock();
	const std::lock_guard<std::mutex> lock(mutex);

	bool result = false;

	_loadIfNotOpen();

	if(kernelModuleFile >= 0) {
		lseek(kernelModuleFile, 0, SEEK_SET); // reset source
		size_t len = read(kernelModuleFile, data, length);
		result = (len == length);
	}

	return result;
}

bool KernelDevice::ReadModuleParameter(const std::string paramName, std::string& output) {
	// should not need a lock
	const std::string pathToParam = pathToModuleParameters + paramName;
	bool result = false;

	std::ifstream inFile{pathToParam};

	if(inFile) {
		inFile >> output;
		result = true;
	}

	return result;
}

void KernelDevice::LockedAccess(std::function<void(int fileID)> f) {
	// the read lock has to be acquired first because
	// there is a risk for a deadlock otherwise!
	const auto fpgaLock = FPGAManager::ScopedReadLock();
	const std::lock_guard<std::mutex> lock(mutex);

	_loadIfNotOpen();

	if(kernelModuleFile >= 0) {
		f(kernelModuleFile);
	}
}
