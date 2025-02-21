#include "imu_x1.hpp"
#include <pthread.h> 

void* process_packet_thread(void* arg) {
    unsigned char* data_buffer = (unsigned char*) arg;  
    process_packet(data_buffer, imu);  
    pthread_exit(NULL);  
}

void process_packet(const unsigned char data_buffer[13], sensor_msgs::Imu& imu) {
    // 1. 0x02의 위치를 탐색함
    int stx_index = -1;
    for (int i = 0; i < 13; ++i) {
        if (data_buffer[i] == STX) {
            stx_index = i;
            break;
        }
    }
    if (stx_index == -1) {
        //ROS_WARN("STX (0x02) not found in the packet");
        return;
    }
    
    std::vector<unsigned char> extracted_data; //패킷 정렬 백터

    // 2. 0x02(STX)부터 고정 패킷 할당 
    extracted_data.push_back(STX);

    // 3. 0x02 이후 패킷 데이터 할당
    for (int i = stx_index + 1; i < 13; ++i) {
        extracted_data.push_back(data_buffer[i]);
    }

    // 4. 0x02 이전의 데이터를 뒤에 추가
    for (int i = 0; i < stx_index; ++i) {
        extracted_data.push_back(data_buffer[i]);
    }

    // 5. 패킷 할당 크기 디버깅
    // if (extracted_data.size() != 13) {
    //     ROS_WARN("Reconstructed data size is not 13 bytes or Invaild packet data!!!");
    // }

    // 6. 재구성된 데이터 출력
    // std::stringstream ss;
    // ss << "Reconstructed Data: ";
    // for (size_t i = 0; i < extracted_data.size(); ++i) {
    //     ss << "0x" << std::setw(2) << std::setfill('0') << std::hex << (int)extracted_data[i] << " ";
    // }
    // ROS_INFO_STREAM(ss.str());

    // 7. 패킷 유효성 검사: 마지막 바이트가 0x03(ETX)인지 확인
    bool is_valid_packet = extracted_data.back() == ETX;
    if (!is_valid_packet) {
        //ROS_WARN_STREAM("\033[96mInvalid data! Last byte is not 0x03 (ETX).\033[0m");
        return;                                                 // If not valid, return and don't process data
    }


    // 8. 센서 타입에 따른 데이터 처리
    unsigned char sensor_type = extracted_data[4]; // 데이터 패킷의 5번째 바이트 (0-based index)
        
    switch (sensor_type) {
        case 0X33:  // 가속도 센서 (ACC)
            {
                acc_x = (int16_t)(((int)(unsigned char)extracted_data[5] | (int)(unsigned char)extracted_data[6] << 8)) / 1000.0;
                acc_y = (int16_t)(((int)(unsigned char)extracted_data[7] | (int)(unsigned char)extracted_data[8] << 8)) / 1000.0;
                acc_z = (int16_t)(((int)(unsigned char)extracted_data[9] | (int)(unsigned char)extracted_data[10] << 8)) / 1000.0;
                
                imu.linear_acceleration.x = acc_x  * 9.80665;
                imu.linear_acceleration.y = acc_y  * 9.80665;
                imu.linear_acceleration.z = acc_z * 9.80665;

                // ROS_INFO_STREAM("\033[1;32macc_x : " << acc_x << ", acc_y : " << acc_y << ", acc_z : " << acc_z << "\033[0m");
                // std::cout << "[DEBUG] ACC: (" << imu.linear_acceleration.x << ", "
                //      << imu.linear_acceleration.y << ", " << imu.linear_acceleration.z << ")" << std::endl;
            }
            break;

        case 0X34:  // 각속도 센서 (GYR)
            {
                gyr_x = (int16_t)(((int)(unsigned char)extracted_data[5] | (int)(unsigned char)extracted_data[6] << 8)) / 10.0;
                gyr_y = (int16_t)(((int)(unsigned char)extracted_data[7] | (int)(unsigned char)extracted_data[8] << 8)) / 10.0;
                gyr_z = (int16_t)(((int)(unsigned char)extracted_data[9] | (int)(unsigned char)extracted_data[10] << 8)) / 10.0;
                
                imu.angular_velocity.x = gyr_x * (M_PI / 180.0);
                imu.angular_velocity.y = gyr_y  * (M_PI / 180.0);
                imu.angular_velocity.z = gyr_z  * (M_PI / 180.0);

                 //ROS_INFO_STREAM("\033[1;33mgyr_x : " << gyr_x << ", gyr_y : " << gyr_y << ", gyr_z : " << gyr_z << "\033[0m");
                // std::cout << "[DEBUG] GYO: (" << imu.angular_velocity.x << ", "
                // << imu.angular_velocity.y << ", " << imu.angular_velocity.z << ")" << std::endl;
            }
            break;            

        case 0X35:  // 오일러 각도 센서 (ANG)
        {
            ang_x = (int16_t)(((int)(unsigned char)extracted_data[5] | (int)(unsigned char)extracted_data[6] << 8)) / 100.0;
            ang_y = (int16_t)(((int)(unsigned char)extracted_data[7] | (int)(unsigned char)extracted_data[8] << 8)) / 100.0;
            ang_z = (int16_t)(((int)(unsigned char)extracted_data[9] | (int)(unsigned char)extracted_data[10] << 8)) / 100.0;

            //ROS_INFO_STREAM("\033[1;34mang_x : " << ang_x << ", ang_y : " << ang_y << ", ang_z : " << ang_z << "\033[0m");

            // **Eular Angle [Degree] to [Radian]**
            double roll = ang_x * M_PI / 180.0;
            double pitch = ang_y * M_PI / 180.0;
            double yaw = ang_z * M_PI / 180.0;

            // **Euler Angle[Radian] to Qutanian **
            tf::Quaternion q;
            q.setRPY(roll, pitch, yaw);

            imu.orientation.w = q.w();
            imu.orientation.x = q.x();
            imu.orientation.y = q.y();
            imu.orientation.z = q.z();

            //std::cout << "[DEBUG] GYO: (" << imu.orientation.w << ", "
            //<< imu.orientation.x << ", " <<  imu.orientation.y << ", "<< imu.orientation.z  << ")" << std::endl;
        }
            break;

        case 0X36:  // 자기 값 (MAG)
            // 자기 값에 대한 처리 (예: Magnitude 값을 읽고 출력)
            {

            }            
            break;

        default:
            // Object index 디버깅
            //ROS_WARN("Unknown sensor type received: %02X", sensor_type); 
            break;
    }
}

// 시리얼 데이터를 읽는 스레드 함수
void* serial_read_thread(void* arg) {
    unsigned char* data_buffer = (unsigned char*) arg;
    while (ros::ok()) {
        if (my_serial.available() >= 13) {
            std::lock_guard<std::mutex> lock(data_mutex);
            my_serial.read(data_buffer, 13);
            new_data_available = true;
        }
        usleep(2500); // 2.5ms대기, 20(20000)ms인경우 패킷값이 엄청 빨리들어옴
    }
    pthread_exit(NULL);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_x1_node");
    ros::NodeHandle nh;

    tf::TransformBroadcaster tf_broadcaster; 

    //tf2::Quaternion tf_orientation;    
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);

    ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("serial_port", serial_port, "/dev/USB0"); ///dev/ttyUSB0
	nh_private.param<std::string>("frame_id", frame_id, "imu_link");
	nh_private.param<int>("baud_rate", baud_rate, 115200);
	nh_private.param<int>("rate_hz", rate_hz, 500);

    ROS_INFO_STREAM("Serial Port is set to: " << serial_port);
    ROS_INFO_STREAM("Baud rate is set to: " << baud_rate);
    ROS_INFO_STREAM("rate Hz is set to: " << rate_hz);
 
    my_serial.setPort(serial_port);
    my_serial.setBaudrate(baud_rate);     
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);  // 타임아웃 설정
    my_serial.setTimeout(to);

    imu.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    imu.angular_velocity_covariance = {0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02};
    imu.linear_acceleration_covariance = {0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04};

    imu.linear_acceleration.x = 0;
    imu.linear_acceleration.y = 0;
    imu.linear_acceleration.z = 0;

    imu.angular_velocity.x = 0;
    imu.angular_velocity.y = 0;
    imu.angular_velocity.z = 0;

    imu.orientation.w = 1;  // 기본 quaternion 설정
    imu.orientation.x = 0;
    imu.orientation.y = 0;
    imu.orientation.z = 0;


    try {
        my_serial.open();  // 시리얼 포트 열기
        ROS_INFO("Serial port opened successfully");
    } catch (serial::IOException& e) {
        ROS_ERROR("Unable to open serial port");
        return -1;
    }

    if (!my_serial.isOpen()) {
        ROS_ERROR("Serial port not open");
        return -1;
    }

    // 시리얼 데이터를 저장할 버퍼
    std::vector<unsigned char> data_buffer(13);
    pthread_t serial_thread;

    // 시리얼 읽기 스레드 생성
    int result = pthread_create(&serial_thread, NULL, serial_read_thread, (void*)data_buffer.data());
    if (result != 0) {
        ROS_ERROR("Error creating serial read thread");
        return -1;
    }

    ros::Rate loop_rate(rate_hz);
    ros::Time loop_start_time = ros::Time::now();

    while (ros::ok()) {
            ros::Time iter_start = ros::Time::now(); // 루프 시작 시간

            imu.header.frame_id = frame_id;
            imu.header.stamp = ros::Time::now();

            imu_pub.publish(imu);

        if (new_data_available) {
            std::lock_guard<std::mutex> lock(data_mutex);
            process_packet(data_buffer.data(), imu);
            new_data_available = false;

       
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(0.0, 0.0, 0.1));
            tf::Quaternion q(
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w
            );
            transform.setRotation(q);
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", frame_id));
        }

        ros::spinOnce();
        loop_rate.sleep();

        // 실제 루프 실행 시간 측정
        //ros::Duration loop_exec_time = ros::Time::now() - iter_start;
        //ROS_WARN_STREAM("\033[1;33m[Loop Execution Time]: " << loop_exec_time.toSec() * 1000 << " ms\033[0m");

        
    }

    // 시리얼 읽기 스레드 종료
    pthread_cancel(serial_thread);
    pthread_join(serial_thread, NULL);

    return 0;
}
