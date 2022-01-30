/*
 * InfluxDBMessageHelper.h
 *
 *  Created on: 5 Jan 2022
 *      Author: SRT
 */

#ifndef INFLUXDBMESSAGEHELPER_H_
#define INFLUXDBMESSAGEHELPER_H_

#include "KafkaMessageProducer.h"
#include "MPU9250.h"
#include <vector>

class InfluxDBMessageHelper {
public:
    InfluxDBMessageHelper(KafkaMessageProducer& kafka, std::string channel = "dummy"): kafka(kafka), channel(channel) {};

    bool sendHumidityAndTemperature(float hum, float temp);
    bool SendMPUData(MPUData& mpuData);
    bool SendMPUDataArray(std::vector<MPUData>& vec);

private:
    KafkaMessageProducer& kafka;
    std::string channel;

};

#endif /* INFLUXDBMESSAGEHELPER_H_ */
