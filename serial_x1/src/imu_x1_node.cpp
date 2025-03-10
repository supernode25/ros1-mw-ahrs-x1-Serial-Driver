// #include "imu_x1.hpp"

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "imu_x1");
//     ros::NodeHandle nh;

//     ImuSensor imu_sensor(nh, "/dev/ttyUSB0", 115200);

//     ros::Rate loop_rate(400);
//     while (ros::ok()) {
//         imu_sensor.readSerialData();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }
