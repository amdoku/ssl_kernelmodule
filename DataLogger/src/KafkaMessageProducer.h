/*
 * KafkaMessageProducer.h
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#ifndef KAFKAMESSAGEPRODUCER_H_
#define KAFKAMESSAGEPRODUCER_H_

// include kafka
#include <librdkafka/rdkafkacpp.h>
// note: use rdkafka++ for lib definition for c++ code!!!

#include <string>
#include <thread>

#include "DeliveryReporterCallback.h"

class KafkaMessageProducer {
public:
	//
	// Create a KafkaMessageProducer for a certain Host/Broker-Address
	KafkaMessageProducer(std::string broker);

	// destroy kafka processing thread
	virtual ~KafkaMessageProducer();

	// set configuration and create required objects
	bool init();

	//
	// Send a message to a specific topic
	bool SendMessage(std::string topic, std::string payload, int64_t time = 0);

private:
	RdKafka::Conf *conf = nullptr;
	RdKafka::Producer *producer = nullptr;
	std::thread *thread = nullptr;
	bool running = false;

	std::string broker;
	DeliveryReporterCallback reporterCallback;

	// thread main loop
	void KafkaPoller();

};

#endif /* KAFKAMESSAGEPRODUCER_H_ */
