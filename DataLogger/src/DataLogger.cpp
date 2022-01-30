#include <iostream>
#include <string>
#include <cstdlib>
#include <cstdio>
#include <csignal>
#include <cstring>
#include <thread>
#include <chrono>
#include <condition_variable>

#include <signal.h>

#include "HDC1000.h"
#include "MPU9250.h"
#include "KafkaMessageProducer.h"
#include "InfluxDBMessageHelper.h"

using namespace std::chrono_literals;


#define DL_MAX_ERROR_MESSAGES 15

static void printErrorMsg(std::string msg, int& count) {
	count ++;
	if(count < DL_MAX_ERROR_MESSAGES) {
		std::cerr << msg << std::endl;
	} else if(count == DL_MAX_ERROR_MESSAGES) {
		std::cerr << "Too many errors! Printing stopped ..." << std::endl;
	}
}

static volatile sig_atomic_t run = 1;

static std::condition_variable cv;
static std::mutex cv_m;

static void sigterm(int sig) {
	run = 0;
	cv.notify_all();
}

static void dataInterrupt(int sig) {
	std::cout << "Data interrupt!" << std::endl;
	cv.notify_all();
}

int main(int argc, char *argv[]) {

	std::string broker = "193.170.192.227";
	std::string topic = "SSL-bucket";

	// https://www.gnu.org/software/libc/manual/html_node/Sigaction-Function-Example.html
	struct sigaction exitAction;

	exitAction.sa_handler = sigterm;
	sigemptyset(&exitAction.sa_mask);
	exitAction.sa_flags = 0;

	// register callbacks
	sigaction(SIGINT, &exitAction, nullptr);
	sigaction(SIGTERM, &exitAction, nullptr);
	

	HDC1000 hdc; // init HDC1000
	MPU9250 mpu; // init MPU9250
	KafkaMessageProducer kafka(broker);
	InfluxDBMessageHelper msg(kafka);

	// register interrupt:
	mpu.RegisterInterrupt(dataInterrupt);

	if (!kafka.init()) {
		std::cerr << "Unable to open connection to server." << std::endl;
		run = 0;
	}


	auto t1 = std::thread([&] {
		float temperature, relativeHumidity;
		int errCount = 0;

		while(run) {
			if (!hdc.Read(temperature, relativeHumidity)) {
				printErrorMsg("Error while reading from HDC.", errCount);
				std::this_thread::sleep_for(1000ms);
				continue;
			}

			bool result = msg.sendHumidityAndTemperature(relativeHumidity, temperature);

			if (result) {
				errCount = 0;
			} else {
				printErrorMsg("Error while sending a HDC message", errCount);
			}
			std::this_thread::sleep_for(100ms); // 10 Hz
		}
	});

	auto t2 = std::thread([&] {
		MPUData mpuData;
		int errCount = 0;

		while(run) {

			if (!mpu.Read(mpuData)) {
				printErrorMsg("Error while reading from MPU.", errCount);
				std::this_thread::sleep_for(1000ms);
				continue;
			}

			bool result = msg.SendMPUData(mpuData);

			if (result) {
				errCount = 0;
			} else {
				printErrorMsg("Error while sending a MPU message", errCount);
			}
			std::this_thread::sleep_for(500ms);
		}
	});

	auto t3 = std::thread([&] {
		std::vector<MPUData> mpuInterruptDataVec;
		std::unique_lock<std::mutex> lk(cv_m);
		int errCount = 0;

		while(run) {
			cv.wait(lk);

			if(!run) {
				break;
			}

			if(!mpu.ReadInterruptBuffer(mpuInterruptDataVec)) {
				printErrorMsg("Error while reading from MPU interrupt.", errCount);
				continue;
			}

			bool result = msg.SendMPUDataArray(mpuInterruptDataVec);

			if (result) {
				errCount = 0;
			} else {
				printErrorMsg("Error while sending a MPU interrupt message", errCount);
			}
		}
	});


	t1.join();
	t2.join();
	t3.join();

	return 0;
}
