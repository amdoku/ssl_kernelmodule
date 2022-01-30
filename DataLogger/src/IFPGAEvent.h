/*
 * IFPGADevice.h
 *
 *  Created on: 25 Nov 2021
 *      Author: SRT
 */

#ifndef FPGA_IFPGAEVENT_H_
#define FPGA_IFPGAEVENT_H_

class IFPGAEvent {
public:
	virtual ~IFPGAEvent() = default;
	virtual void onFPGARelease() = 0;
	virtual void onFPGAAcquire() = 0;
};

#endif /* FPGA_IFPGAEVENT_H_ */
