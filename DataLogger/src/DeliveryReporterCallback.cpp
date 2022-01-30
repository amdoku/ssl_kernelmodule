/*
 * DeliveryReporterCallback.cpp
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#include "DeliveryReporterCallback.h"

#include <iostream>

void DeliveryReporterCallback::dr_cb(RdKafka::Message &message) {

	/* If message.err() is non-zero the message delivery failed permanently
	 * for the message. */
	if (message.err())
		std::cerr << "% Message delivery failed: " << message.errstr()
				<< std::endl;

}
