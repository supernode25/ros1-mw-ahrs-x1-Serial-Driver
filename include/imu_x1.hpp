#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <chrono>
#include <pthread.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>

using namespace std::chrono;

#ifndef IMU_X1_HPP
#define IMU_X1_HPP

#define DeviceID 0x01
#define STX 0x02
#define ETX 0x03
#define Command 0xF0
// #define ACC 0x33
// #define GYR 0x34
// #define ANG 0x35
// #define MAG 0x36

#define DEG2RAD( a ) ( (a) * (M_PI/180.0f) )
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

// 변수를 선언 (예: 가속도 센서의 x, y, z 값)
int16_t acc_x = 0, acc_y = 0, acc_z = 0;
int16_t gyr_x = 0, gyr_y = 0, gyr_z = 0;
int16_t ang_x = 0, ang_y = 0, ang_z = 0;

std::string serial_port;
std::string frame_id;
int baud_rate;
int rate_hz;
bool run;

high_resolution_clock::time_point last_recv_time;
high_resolution_clock::time_point last_pub_time;

sensor_msgs::Imu imu;
serial::Serial my_serial;
pthread_t thread_id;
void process_packet(const unsigned char data_buffer[13], sensor_msgs::Imu& imu);
//std::vector<unsigned char> data_buffer(13);
std::mutex data_mutex;
//unsigned char data_buffer[13] = {0,};
bool new_data_available = false;

// std::vector<unsigned char> extracted_data; // 패킷 정렬 벡터

#endif // IMU_X1_HPP