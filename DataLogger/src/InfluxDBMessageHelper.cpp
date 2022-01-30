/*
 * InfluxDBMessageHelper.cpp
 *
 *  Created on: 5 Jan 2022
 *      Author: SRT
 */

#include "InfluxDBMessageHelper.h"
#include "TimeHelper.h"


bool InfluxDBMessageHelper::sendHumidityAndTemperature(float hum, float temp) {
    int64_t currentTime = getCurrentTime();
    return kafka.SendMessage(channel,
					"weather relativeHumidity=" + std::to_string(hum) + "," +
					"temperature=" + std::to_string(temp) + " " +
					std::to_string(currentTime)
			);
}


static inline std::string BuildMPUString(std::string dest, MPUData& mpuData, bool iter) {
    MPUAxis& accel = mpuData.accel;
    MPUAxis& gyro = mpuData.gyro;
    MPUAxis& mag = mpuData.mag;

    return dest +
        " accelX=" + std::to_string(accel.x) + "," +
        "accelY=" + std::to_string(accel.y) + "," +
        "accelZ=" + std::to_string(accel.z) + "," +

        "gyroX=" + std::to_string(gyro.x) + "," +
        "gyroY=" + std::to_string(gyro.y) + "," +
        "gyroZ=" + std::to_string(gyro.z) + "," +

        "magX=" + std::to_string(mag.x) + "," +
        "magY=" + std::to_string(mag.y) + "," +
        "magZ=" + std::to_string(mag.z) + "," +

        "iter=" + std::to_string(iter) + " " +

        std::to_string(mpuData.timestamp_ns);
}


bool InfluxDBMessageHelper::SendMPUData(MPUData& mpuData) {
    //int64_t currentTime = std::chrono::system_clock::now().time_since_epoch().count();

    return kafka.SendMessage(channel, BuildMPUString("MPU", mpuData, false));
}


bool InfluxDBMessageHelper::SendMPUDataArray(std::vector<MPUData>& vec)
{
    std::string msg = "";
    for(MPUData& mpu : vec) {
        msg += BuildMPUString("MPU", mpu, true) + "\n";
    }
    return kafka.SendMessage(channel, msg);
}
