/*
 * DeliveryReporterCallback.h
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#ifndef DELIVERYREPORTERCALLBACK_H_
#define DELIVERYREPORTERCALLBACK_H_

// include kafka
#include <librdkafka/rdkafkacpp.h>
// note: use rdkafka++ for lib definition for c++ code!!!

class DeliveryReporterCallback final: public RdKafka::DeliveryReportCb {
public:
	void dr_cb(RdKafka::Message &message);
};

#endif /* DELIVERYREPORTERCALLBACK_H_ */
