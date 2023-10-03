
#include <iostream>
#include <signal.h>

#include <string>
#include <vector>
#include <fstream>

#include "oscpack/include/osc/OscReceivedElements.h"
#include "oscpack/include/osc/OscPacketListener.h"
#include "oscpack/include/ip/UdpSocket.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <thread>

#include <string>
// receive sensor packets, parse, and then publish IMU message

// Port number set from NGIMU GUI; set the received ip address to be the ip address of this PC
#define PORT1 8101
#define PORT2 8102
#define PORT3 8103
#define PORT4 8104

float w, x, y, z;
sensor_msgs::msg::Imu imuData_1;
sensor_msgs::msg::Imu imuData_2;
sensor_msgs::msg::Imu imuData_3;
sensor_msgs::msg::Imu imuData_4;

geometry_msgs::msg::Vector3 eulerAngles;     // check the angle sequence later
geometry_msgs::msg::Vector3 battery_1_State; // check the angle sequence later

geometry_msgs::msg::Quaternion Voltage_FSR; // voltage of Force sensing resistors

class ExamplePacketListener : public osc::OscPacketListener
{
public:
    int port_number = 8000;

protected:
    virtual void ProcessMessage(const osc::ReceivedMessage &m,
                                const IpEndpointName &remoteEndpoint)
    {
        try
        {
            // example of parsing single messages. osc::OsckPacketListener
            // handles the bundle traversal.
            if (strcmp(m.AddressPattern(), "/quaternion") == 0)
            {
                // example #1 -- argument stream interface
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();

                args >> w >> x >> y >> z >> osc::EndMessage;

                switch (port_number)
                {
                case PORT1:
                    imuData_1.orientation.x = x;
                    imuData_1.orientation.y = y;
                    imuData_1.orientation.z = z;
                    imuData_1.orientation.w = w;
                    imuData_1.header.frame_id = std::to_string(port_number);
                    // imuData_1.header.stamp.sec = ; 
                    // imuData_1.header.stamp.nanosec = ;
                    break;

                case PORT2:
                    imuData_2.orientation.x = x;
                    imuData_2.orientation.y = y;
                    imuData_2.orientation.z = z;
                    imuData_2.orientation.w = w;
                    imuData_2.header.frame_id = std::to_string(port_number);

                    break;
                case PORT3:
                    imuData_3.orientation.x = x;
                    imuData_3.orientation.y = y;
                    imuData_3.orientation.z = z;
                    imuData_3.orientation.w = w;
                    imuData_3.header.frame_id = std::to_string(port_number);
                    break;

                case PORT4:
                    imuData_4.orientation.x = x;
                    imuData_4.orientation.y = y;
                    imuData_4.orientation.z = z;
                    imuData_4.orientation.w = w;
                    imuData_4.header.frame_id = std::to_string(port_number);
                    break;
                }
            }
            else if (strcmp(m.AddressPattern(), "/sensors") == 0)
            {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float gx, gy, gz, ax, ay, az, mx, my, mz, b;
                args >> gx >> gy >> gz >> ax >> ay >> az >> mx >> my >> mz >> b >> osc::EndMessage;

                switch (port_number)
                {

                case PORT1:
                    imuData_1.linear_acceleration.x = ax;
                    imuData_1.linear_acceleration.y = ay;
                    imuData_1.linear_acceleration.z = az;
                    // gyroscope
                    imuData_1.angular_velocity.x = gx;
                    imuData_1.angular_velocity.y = gy;
                    imuData_1.angular_velocity.z = gz;
                    break;
                case PORT2:
                    imuData_2.linear_acceleration.x = ax;
                    imuData_2.linear_acceleration.y = ay;
                    imuData_2.linear_acceleration.z = az;
                    // gyroscope
                    imuData_2.angular_velocity.x = gx;
                    imuData_2.angular_velocity.y = gy;
                    imuData_2.angular_velocity.z = gz;
                    break;
                case PORT3:
                    imuData_3.linear_acceleration.x = ax;
                    imuData_3.linear_acceleration.y = ay;
                    imuData_3.linear_acceleration.z = az;
                    // gyroscope
                    imuData_3.angular_velocity.x = gx;
                    imuData_3.angular_velocity.y = gy;
                    imuData_3.angular_velocity.z = gz;
                    break;

                case PORT4:
                    imuData_4.linear_acceleration.x = ax;
                    imuData_4.linear_acceleration.y = ay;
                    imuData_4.linear_acceleration.z = az;
                    // gyroscope
                    imuData_4.angular_velocity.x = gx;
                    imuData_4.angular_velocity.y = gy;
                    imuData_4.angular_velocity.z = gz;
                    break;
                } // std::cout << "received '/sensors' message with arguments: "
                //<< gx << " " << gy << " " << gz << "\n";
            }
            else if (strcmp(m.AddressPattern(), "/euler") == 0)
            {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float ex, ey, ez;
                args >> ex >> ey >> ez >> osc::EndMessage;
                eulerAngles.x = ex;
                eulerAngles.y = ey;
                eulerAngles.z = ez;
            }
            else if (strcmp(m.AddressPattern(), "/battery") == 0)
            {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float level, time_left, voltage, current;
                std::string state;
                args >> level >> time_left >> voltage >> state >> osc::EndMessage;
                switch (port_number)
                {

                case PORT1:
                    battery_1_State.x = level;
                    battery_1_State.y = time_left;
                    battery_1_State.z = voltage;
                    break;
                }
            }

            else if (strcmp(m.AddressPattern(), "/analogue") == 0)
            {
                osc::ReceivedMessageArgumentStream args = m.ArgumentStream();
                float voltages[8]; // 8 analogue channels on NGIMU
                args >> voltages[0] >> voltages[1] >> voltages[2] >> voltages[3] >> voltages[4] >> voltages[5] >> voltages[6] >> voltages[7] >> osc::EndMessage;
                // std::cout << "voltage 1: " << voltages[0] << "voltage 2: " << voltages[1]
                //           << "voltage 3: " << voltages[2] << "voltage 4: " << voltages[3] << std::endl;
                Voltage_FSR.x = voltages[0];
                Voltage_FSR.y = voltages[1];
                Voltage_FSR.z = voltages[2];
                Voltage_FSR.w = voltages[3];
                switch (port_number)
                {
                case PORT1:
                    imuData_1.orientation_covariance[0] = voltages[0];
                    imuData_1.orientation_covariance[1] = voltages[1];
                    imuData_1.orientation_covariance[2] = voltages[2];
                    imuData_1.orientation_covariance[3] = voltages[3];

                    break;
                case PORT2:

                    imuData_2.orientation_covariance[0] = voltages[0];
                    imuData_2.orientation_covariance[1] = voltages[1];
                    imuData_2.orientation_covariance[2] = voltages[2];
                    imuData_2.orientation_covariance[3] = voltages[3];

                    break;
                case PORT3:
                    imuData_3.orientation_covariance[0] = voltages[0];
                    imuData_3.orientation_covariance[1] = voltages[1];
                    imuData_3.orientation_covariance[2] = voltages[2];
                    imuData_3.orientation_covariance[3] = voltages[3];

                    break;

                case PORT4:
                    imuData_4.orientation_covariance[0] = voltages[0];
                    imuData_4.orientation_covariance[1] = voltages[1];
                    imuData_4.orientation_covariance[2] = voltages[2];
                    imuData_4.orientation_covariance[3] = voltages[3];

                    break;
                };
            }

            // TODO: check euler, and time stamp
            //  raise(SIGINT);
        }
        catch (osc::Exception &e)
        {
            // any parsing errors such as unexpected argument types, or
            // missing arguments get thrown as exceptions.
            std::cout << "error while parsing message: "
                      << m.AddressPattern() << ": " << e.what() << "\n";
        }
    }
};

void runningListener(int PORT_number)
{
    ExamplePacketListener listener;
    listener.port_number = PORT_number;

    UdpListeningReceiveSocket udp_listener(
        IpEndpointName(IpEndpointName::ANY_ADDRESS, PORT_number), // all available interfaces.
        &listener);

    // UdpListeningReceiveSocket udp_listener(
    //     IpEndpointName("192.168.0.102", PORT), // all available interfaces.
    //     &listener);

    udp_listener.Run(); //
    // udp_listener.RunUntilSigInt();
};

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ngimu_wifi");

    auto imuPub1 = node->create_publisher<sensor_msgs::msg::Imu>("/ngimu/imu_sensor_8101", 5);
    auto imuPub2 = node->create_publisher<sensor_msgs::msg::Imu>("/ngimu/imu_sensor_8102", 5);
    auto imuPub3 = node->create_publisher<sensor_msgs::msg::Imu>("/ngimu/imu_sensor_8103", 5);
    auto imuPub4 = node->create_publisher<sensor_msgs::msg::Imu>("/ngimu/imu_sensor_8104", 5);

    auto eulerPub = node->create_publisher<geometry_msgs::msg::Vector3>("/ngimu/imu_euler", 5);
    auto batteryPub = node->create_publisher<geometry_msgs::msg::Vector3>("/ngimu/imu_battery", 5);

    auto voltagePub = node->create_publisher<geometry_msgs::msg::Quaternion>("/ngimu/voltage_Fsr", 5);

    rclcpp::Rate loop_rate(400); // IMU reading out maximum at 400HZ

    // data buffer that write into text file.
    std::vector<std::string> list;

    std::thread th1(runningListener, PORT1); // OSC pack listener continues to listen no matter what, has to put it on a different thread
    th1.detach();

    std::thread th2(runningListener, PORT2); // OSC pack listener continues to listen no matter what, has to put it on a different thread
    th2.detach();

    // std::thread th3(runningListener, PORT3); // OSC pack listener continues to listen no matter what, has to put it on a different thread
    // th3.detach();

    // std::thread th4(runningListener, PORT4); // OSC pack listener continues to listen no matter what, has to put it on a different thread
    // th4.detach();
    // // keep listening

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        // std::cout << "quaternion: " << w << " " << x << " " << y << " " << z << "\n";
        imuPub1->publish(imuData_1);
        imuPub2->publish(imuData_2);
        imuPub3->publish(imuData_3);
        imuPub4->publish(imuData_4);

        eulerPub->publish(eulerAngles);
        batteryPub->publish(battery_1_State);

        voltagePub->publish(Voltage_FSR);

        loop_rate.sleep();

        // list.push_back(std::to_string(w) + " " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z));
    }

    // write into text file.
    std::ofstream outFile("NGIMUdata.txt");
    if (!list.empty())
    {
        for (const auto &line : list)
            outFile << line << "\n";
    }
    return 0;
}
