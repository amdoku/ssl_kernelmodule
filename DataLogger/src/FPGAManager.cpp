/*
 * FPGAManager.cpp
 *
 *  Created on: 22 Nov 2021
 *      Author: SRT
 */

#include "FPGAManager.h"

#include <iostream>

//
// Init static members
//

RWMutex FPGAManager::mutex;
bool FPGAManager::fpgaAcquired = false;
bool FPGAManager::reconfigurationInProgress = false;
FpgaRegion FPGAManager::fpga("Data Logger", FPGAManager::ReconfigRequest, FPGAManager::ReconfigDone);
std::list<IFPGAEvent*> FPGAManager::callbacksFPGAEvents;

//
// Init functions
//

//
// Acquire FPGA and kernel driver
void FPGAManager::_unsafeFPGAAcquire()
{
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	fpga.Acquire();
	fpgaAcquired = true;

	for(IFPGAEvent* f : callbacksFPGAEvents)
	{
		f->onFPGAAcquire();
	}
}

//
// Release FPGA and kernel driver
void FPGAManager::_unsafeFPGARelease()
{
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	fpgaAcquired = false;

	for(IFPGAEvent* f : callbacksFPGAEvents)
	{
		f->onFPGARelease();
	}

	fpga.Release();
}


void FPGAManager::ReconfigRequest() {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const WriteLock lock(mutex);

	reconfigurationInProgress = true;

	if(fpgaAcquired) {
		_unsafeFPGARelease();
	}
}

void FPGAManager::ReconfigDone() {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const WriteLock lock(mutex);

	reconfigurationInProgress = false;

	if(!fpgaAcquired) {
		_unsafeFPGAAcquire();
	}

}


void FPGAManager::registerEvent(IFPGAEvent* event)
{
	if(event == nullptr) {
		return;
	}

	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const WriteLock lock(mutex);

	if(!fpgaAcquired && !reconfigurationInProgress) {
		_unsafeFPGAAcquire();
	}

	callbacksFPGAEvents.push_back(event);

}

void FPGAManager::unregisterEvent(IFPGAEvent* event)
{
	if(event == nullptr) {
		return;
	}

	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const WriteLock lock(mutex);

	callbacksFPGAEvents.remove(event);

	if(fpgaAcquired && callbacksFPGAEvents.empty()) {
		std::cout << __FILE__ << ": " << __FUNCTION__ << " - No more events -> releasing FPGA" << std::endl;
		_unsafeFPGARelease();
	}

}


void FPGAManager::EventInitCall(IFPGAEvent* event) {
	std::cout << __FILE__ << ": " << __FUNCTION__ << std::endl;

	const ReadLock lock = ScopedReadLock();

	if(fpgaAcquired) {
		event->onFPGAAcquire();
	}
}

bool FPGAManager::IsFPGAAcquired()
{
	return fpgaAcquired;
}

