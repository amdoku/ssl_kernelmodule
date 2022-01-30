/*
 * KafkaMessageProducer.cpp
 *
 *  Created on: 31 Oct 2021
 *      Author: SRT
 */

#include "KafkaMessageProducer.h"

#include <iostream>
#include <chrono>

KafkaMessageProducer::KafkaMessageProducer(std::string broker) :
		broker(broker) {
}

KafkaMessageProducer::~KafkaMessageProducer() {
	if (producer != nullptr) {
		running = false;
		thread->join(); // wait for thread to complete

		delete thread;
		thread = nullptr;
		delete producer;
		producer = nullptr;
	}
}

bool KafkaMessageProducer::init() {
	std::string errstr;
	bool result = true;

	if (producer == nullptr) {

		RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);

		if (conf->set("security.protocol", "ssl", errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		// set server cert
		if (conf->set("ssl.ca.location", "/etc/datalogger/certs/ca-cert", errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		// set client cert
		if (conf->set("ssl.certificate.location", "/etc/datalogger/certs/client_ssl_client.pem",
				errstr) != RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		if (conf->set("ssl.key.location", "/etc/datalogger/certs/client_ssl_client.key", errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		if (conf->set("ssl.key.password", "C#127B", errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		// set connection information
		if (conf->set("bootstrap.servers", broker, errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		// set reporter callback
		if (conf->set("dr_cb", &reporterCallback, errstr)
				!= RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			result = false;
		}

		/*if (conf->set("debug", "all", errstr) != RdKafka::Conf::CONF_OK) {
			std::cerr << errstr << std::endl;
			exit(1);
		}*/

		// create producer
		producer = RdKafka::Producer::create(conf, errstr);

		if (producer == nullptr) {
			std::cerr << errstr << std::endl;
			result = false;
		} else {
			// start thread
			running = true;
			thread = new std::thread(&KafkaMessageProducer::KafkaPoller, this);
		}

		delete conf;
	}

	return result;
}

bool KafkaMessageProducer::SendMessage(std::string topic, std::string payload,
		int64_t time) {

	if (producer == nullptr) {
		return false;
	}

	/*
	 * Send/Produce message.
	 * This is an asynchronous call, on success it will only
	 * enqueue the message on the internal producer queue.
	 * The actual delivery attempts to the broker are handled
	 * by background threads.
	 * The previously registered delivery report callback
	 * is used to signal back to the application when the message
	 * has been delivered (or failed permanently after retries).
	 */
	RdKafka::ErrorCode err = producer->produce(topic,
	/* Any Partition: the builtin partitioner will be
	 * used to assign the message to a topic based
	 * on the message key, or random partition if
	 * the key is not set. */
	RdKafka::Topic::PARTITION_UA,
	/* Make a copy of the value */
	RdKafka::Producer::RK_MSG_COPY,
	/* payload */
	const_cast<char*>(payload.c_str()), payload.size(),
	/* Key */
	NULL, 0,
	/* Timestamp (defaults to current time) */
	time,
	/* Per-message opaque value passed to
	 * delivery report */
	NULL);

	if (err != RdKafka::ERR_NO_ERROR) {
		std::cerr << "% Failed to produce to topic " << topic << ": "
				<< RdKafka::err2str(err) << std::endl;

		if (err == RdKafka::ERR__QUEUE_FULL) {
			std::cerr << "queue full!" << std::endl;
		}

		return false;
	}

	return true;
}

//
// Thread loop to run kafka
void KafkaMessageProducer::KafkaPoller() {
	using namespace std::chrono_literals;

	while (running) {
		producer->poll(100);

		if (!running)
			break;

		std::this_thread::sleep_for(1ms);
	}

	std::cout << "KafkaPoller: finished" << std::endl;
}
