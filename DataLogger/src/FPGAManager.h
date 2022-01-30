/*
 * FPGAManager.h
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */

#ifndef FPGAMANAGER_H_
#define FPGAMANAGER_H_

#if defined(__APPLE__) || defined(_WIN32) || defined(_WIN64)
	#include "LibFPGARegionMock.h"
#else
	#include <libfpgaregion.h>
#endif

#include <shared_mutex>
#include <list>

#include "IFPGAEvent.h"

using RWMutex = std::shared_timed_mutex;
using ReadLock  = std::shared_lock<RWMutex>;
using WriteLock = std::unique_lock<RWMutex>;

class FPGAManager {
private:
	static FpgaRegion fpga;
	static bool fpgaAcquired, reconfigurationInProgress;
	static std::list<IFPGAEvent*> callbacksFPGAEvents;

	static RWMutex mutex;

	//
	// FPGA configuration request callback -> close driver file
	static void ReconfigRequest();

	//
	// FPGA reconfiguration done callback
	static void ReconfigDone();

	FPGAManager() = default;
	~FPGAManager() = default;

	//
	// Acquire FPGA and kernel driver
	static void _unsafeFPGAAcquire();

	//
	// Release FPGA and kernel driver
	static void _unsafeFPGARelease();


	//
	// Release FPGA and kernel driver
	static void fpgaAcquire();

	//
	// Release FPGA and kernel driver
	static void fpgaRelease();

public:

	//
	// Register for FPGA Events
	static void registerEvent(IFPGAEvent* event);

	//
	// Unregister events
	static void unregisterEvent(IFPGAEvent* event);

	//
	// Call onFPGAAquire if the FPGA is locked
	// can be used inside a scoped ReadLock
	static void EventInitCall(IFPGAEvent* event);

	//
	// Checks whether the FPGA is acquired
	static bool IsFPGAAcquired();

	//
	// return a scoped readlock for FPGA access
	// See here for reference: https://stackoverflow.com/questions/39185420/is-there-a-shared-lock-guard-and-if-not-what-would-it-look-like
	static ReadLock ScopedReadLock() {
		return ReadLock(mutex);
	}

};

#endif /* FPGAMANAGER_H_ */
