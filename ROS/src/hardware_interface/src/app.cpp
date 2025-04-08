#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <functional>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>
///////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "odometry.hpp"
#include "nlohmann/json.hpp"

#define PACKET_START_CHARACTER '\r'
#define PACKET_END_CHARACTER '\n'

#define LIMIT(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SIGN(x) ((x > 0) ? 1 : -1)

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace boost::asio;
using namespace std;
using json = nlohmann::json;

class robot_base_node : public rclcpp::Node {
    Odometry wheel_odom;
    tf2::Quaternion wheel_odom_quaternion;
    bool command_received_ = false;
    rclcpp::Time last_command_time_;
    const double WATCHDOG_TIMEOUT = 0.5;

    public:
        // for transferring the parameter value into a variable for use inside the code
        rclcpp::Parameter _velocity_input_topic;
        rclcpp::Parameter _usb_port;
        // for using inside the code, the value held by these parameters
        std::string velocity_input_topic_;
        std::string usb_port_;
        boost::asio::io_context io_context_;
        boost::asio::serial_port serial_port_;
        boost::asio::streambuf rx_buffer_;
        boost::system::error_code ec;

        void THREAD__serial_reader()
        {
            while(rclcpp::ok())
            {
                boost::system::error_code _ec;
                std::size_t _bytes_transferred = boost::asio::read_until(this->serial_port_, this->rx_buffer_, PACKET_END_CHARACTER, _ec);

                if(_ec)
                {
                    RCLCPP_ERROR(this->get_logger(), "UART receiver error");
                    break;
                }
                else if(_bytes_transferred > 0)
                {
                    std::istream is(&this->rx_buffer_);
                    std::string _received_line;
                    std::getline(is, _received_line); // extract till \n
                    if(_received_line[0] == PACKET_START_CHARACTER)
                    {
                        // RCLCPP_INFO(this->get_logger(), "%s", &_received_line[0]);
                        
                        rclcpp::Time _now = this->get_clock()->now();

                        auto odometry_message = nav_msgs::msg::Odometry();
                        odometry_message.header.stamp = _now;
                        odometry_message.header.frame_id = "odom"; // only for the sake of not leaving the frame_id's as null
                        odometry_message.child_frame_id = "base_footprint"; // because the proper odometry is actually published by the ekf with appropriate frame_id's
                        // odometry_message.header.frame_id = "base_link";
                        // odometry_message.child_frame_id = "left_wheel_link";
                        // ekf doesn't care about this ^^^, but make changes to the URDF
                        // for including this link in the tf2 tree before setting any name here...
                        // else, just comment out if this is being used by ekf for releasing filtered odometry on "odom" frame

                        if(from_hardware_node.find("rbf") != from_hardware_node.end())
                        {
                            from_hardware_node["rbf"].is_object();
                            if(from_hardware_node["rbf"].find("wf") != from_hardware_node["rbf"].end())
                            {
                                from_hardware_node["rbf"]["wf"].is_object();
                                if(from_hardware_node["rbf"]["wf"].find("v") != from_hardware_node["rbf"]["wf"].end())
                                {
                                    from_hardware_node["rbf"]["wf"]["v"].is_array();
                                    if(from_hardware_node["rbf"]["wf"]["v"].size() >= 2)
                                    {
                                        // Calculate the wheel odometry, and publish to ROS.
                                        wheel_odom.setWheelParams(WHEEL_SEPARATION, WHEEL_RADIUS, WHEEL_RADIUS);
                                        wheel_odom.updateFromVelocity
                                        (
                                            (-1.0f * from_hardware_node["rbf"]["wf"]["v"][0].get<float>() * WHEEL_CIRCUMFERENCE), // rotations-per-second --> meters-per-second
                                            (-1.0f * from_hardware_node["rbf"]["wf"]["v"][1].get<float>() * WHEEL_CIRCUMFERENCE)
                                        );
                                        double _Position_X = wheel_odom.getX(); // aka cartesian coordinates
                                        double _Position_Y = wheel_odom.getY();
                                        wheel_odom_quaternion.setRPY(0.0, 0.0, wheel_odom.getHeading());
                                        double _LinearVelocity_X = wheel_odom.getLinear();
                                        double _AngularVelocity_Z = wheel_odom.getAngular();

                                        odometry_message.pose.pose.position.x = _Position_X;
                                        odometry_message.pose.pose.position.y = _Position_Y;
                                        odometry_message.pose.pose.orientation.w = wheel_odom_quaternion.w();
                                        odometry_message.pose.pose.orientation.x = wheel_odom_quaternion.x();
                                        odometry_message.pose.pose.orientation.y = wheel_odom_quaternion.y();
                                        odometry_message.pose.pose.orientation.z = wheel_odom_quaternion.z();
                                        odometry_message.twist.twist.linear.x = _LinearVelocity_X;
                                        odometry_message.twist.twist.angular.z = _AngularVelocity_Z;
                                    }
                                }
                            }
                        }
                        wheel_odometry_publisher_->publish(odometry_message);

                        // adjust the IMU data, and publish to ROS
                        auto imu_raw_message = sensor_msgs::msg::Imu();
                        auto imu_mag_message = sensor_msgs::msg::MagneticField();
                        imu_raw_message.header.stamp = _now;
                        imu_mag_message.header.stamp = _now;

                        imu_raw_message.linear_acceleration.x = 0.0;
                        imu_raw_message.linear_acceleration.y = 0.0;
                        imu_raw_message.linear_acceleration.z = 0.0;
                        imu_raw_message.angular_velocity.x = 0.0;
                        imu_raw_message.angular_velocity.y = 0.0;
                        imu_raw_message.angular_velocity.z = 0.0;
                        imu_mag_message.magnetic_field.x = 0.0;
                        imu_mag_message.magnetic_field.y = 0.0;
                        imu_mag_message.magnetic_field.z = 0.0;

                        if(from_hardware_node.find("rbf") != from_hardware_node.end())
                        {
                            from_hardware_node["rbf"].is_object();
                            if(from_hardware_node["rbf"].find("imu") != from_hardware_node["rbf"].end())
                            {
                                from_hardware_node["rbf"]["imu"].is_object();
                                if(from_hardware_node["rbf"]["imu"].find("a") != from_hardware_node["rbf"]["imu"].end())
                                {
                                    from_hardware_node["rbf"]["imu"]["a"].is_array();
                                    if(from_hardware_node["rbf"]["imu"]["a"].size() >= 3)
                                    {
                                        imu_raw_message.linear_acceleration.x = (-1.0f * from_hardware_node["rbf"]["imu"]["a"][0].get<float>());
                                        imu_raw_message.linear_acceleration.y = from_hardware_node["rbf"]["imu"]["a"][1].get<float>();
                                        imu_raw_message.linear_acceleration.z = from_hardware_node["rbf"]["imu"]["a"][2].get<float>();
                                    }
                                    if(from_hardware_node["rbf"]["imu"]["g"].size() >= 3)
                                    {
                                        imu_raw_message.angular_velocity.x = from_hardware_node["rbf"]["imu"]["g"][0].get<float>();
                                        imu_raw_message.angular_velocity.y = from_hardware_node["rbf"]["imu"]["g"][1].get<float>();
                                        imu_raw_message.angular_velocity.z = from_hardware_node["rbf"]["imu"]["g"][2].get<float>();
                                    }
                                    if(from_hardware_node["rbf"]["imu"]["m"].size() >= 3)
                                    {
                                        imu_mag_message.magnetic_field.x = from_hardware_node["rbf"]["imu"]["m"][0].get<float>();
                                        imu_mag_message.magnetic_field.y = from_hardware_node["rbf"]["imu"]["m"][1].get<float>();
                                        imu_mag_message.magnetic_field.z = from_hardware_node["rbf"]["imu"]["m"][2].get<float>();
                                    }
                                }
                            }
                        }
                        imu_raw_publisher_->publish(imu_raw_message);
                        imu_mag_publisher_->publish(imu_mag_message);
                    }
                }
            }
        }

        robot_base_node() : Node("hardware_interface_ROS_node"), io_context_(), serial_port_(io_context_)  // Initialize io_context_ and serial_port_ in constructor
        {
            this->declare_parameter("velocity_input_topic", "cmd_vel");
            _velocity_input_topic = this->get_parameter("velocity_input_topic");
            velocity_input_topic_ = _velocity_input_topic.as_string();
            
            this->declare_parameter("usb_port", "/dev/ttyUSB*");
            _usb_port = this->get_parameter("usb_port");
            usb_port_ = _usb_port.as_string();

            serial_port_.open(usb_port_, ec);

            // ROS subscribers
            cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                velocity_input_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&robot_base_node::cmd_vel_callback, this, _1));

            // ROS publishers
            wheel_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
                "odom/wheel_encoder", rclcpp::SystemDefaultsQoS());
            imu_raw_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
                "imu/data_raw", rclcpp::SystemDefaultsQoS());
            imu_mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
                "imu/mag", rclcpp::SystemDefaultsQoS());

            watchdog_timer_ = this->create_wall_timer(
                100ms, std::bind(&robot_base_node::cmd_vel_timeout, this));

            hardware_node_data_sender_timer_ = this->create_wall_timer(
                50ms, std::bind(&robot_base_node::hardware_node_data_sender_callback, this));

            serial_port_.set_option(serial_port_base::baud_rate(460800));
            serial_port_.set_option(serial_port_base::character_size(8));
            serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
        }
        ~robot_base_node()
        {
            if(serial_port_.is_open())
            {
                serial_port_.close(ec);
            }
        }

    private:
        const double WHEEL_RADIUS = 0.100;  // meters
        const double WHEEL_CIRCUMFERENCE = (2 * M_PI * WHEEL_RADIUS);
        const double WHEEL_SEPARATION = 0.555;  // meters
        double NEW_command_linear_X = 0, NEW_command_angular_Z = 0;
        rclcpp::Time previous_time_;
        rclcpp::TimerBase::SharedPtr watchdog_timer_;
        rclcpp::TimerBase::SharedPtr hardware_node_data_sender_timer_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr imu_mag_publisher_;
        json to_hardware_node = 
        {
            {"rbc", // rbc = robot base control
                {"wc", // wc = wheel control
                    {
                        "t", // t = torque
                        {
                            0.0, 0.0 // 2 motors
                        },
                        "v", // v = velocity
                        {
                            0.0, 0.0 // 2 motors
                        },
                        "p", // p = position
                        {
                            0.0, 0.0 // 2 motors
                        }
                    }
                }
            },
            {"rpc", // rpc = robot payload control
                {"cc", // cc = cleaner control
                    {"t", // t = torque
                        {
                            0.0, 0.0 // 2 motors
                        }
                    }
                }
            }
        };
        json from_hardware_node = 
        {
            {"rbf", // rbc = robot base feedback
                {"wf", // wf = wheel feedback
                    {
                        "t", // t = torque
                        {
                            0.0, 0.0 // 2 motors
                        },
                        "v", // v = velocity
                        {
                            0.0, 0.0 // 2 motors
                        },
                        "p", // p = position
                        {
                            0.0, 0.0 // 2 motors
                        }
                    }
                },
                {"imu", // imu = inertial measurement unit
                    {"a", // a = accelerometer
                        {
                            0.0, 0.0, 0.0 // 3 axes
                        }
                    },
                    {"g", // g = gyroscope
                        {
                            0.0, 0.0, 0.0 // 3 axes
                        }
                    },
                    {"m", // m = magnetometer
                        {
                            0.0, 0.0, 0.0 // 3 axes
                        }
                    }
                }
            },
            {"rpf", // rpc = robot payload feedback
                {"ls", // ls = limit switch
                    {
                        false, false // 2 switches
                    }
                },
                {"ds", // ds = distance sensor
                    {
                        0.0 // 1 sensor
                    }
                }
            }
        };

        void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg) // receive the velocity commands from ROS node
        {
            last_command_time_ = this->get_clock()->now();
            
            NEW_command_linear_X = cmd_vel_msg->linear.x;
            NEW_command_angular_Z = cmd_vel_msg->angular.z;
        }

        void cmd_vel_timeout() // check if the velocity command messages from ROS have stopped arriving
        {
            rclcpp::Time now = this->get_clock()->now();
            if ((now.seconds() - last_command_time_.seconds()) > WATCHDOG_TIMEOUT)
            {
                NEW_command_linear_X = 0.0;
                NEW_command_angular_Z = 0.0;
            }
        }

        void hardware_node_data_sender_callback()  // send all data to the hardware
        {
            char packet[1024] = {'\0'};
            sprintf(
                packet,
                "{\"rbc\":{\"wc\":{\"t\":[0.0,0.0],\"v\":[%.2f,%.2f],\"p\":[0.0,0.0]}}},{\"rpc\":{\"cc\":{\"t\":[0.0,0.0]}}}\n",
                // reserved for rbc/wc/t
                (-1.0f * (NEW_command_linear_X - (NEW_command_angular_Z * WHEEL_SEPARATION / 2)) / WHEEL_CIRCUMFERENCE), (-1.0f * (NEW_command_linear_X + (NEW_command_angular_Z * WHEEL_SEPARATION / 2)) / WHEEL_CIRCUMFERENCE)
                // reserved for rbc/wc/p
                // reserved for rpc/cc/t
            );
            write(serial_port_, buffer(packet)); // Send over serial
            // cout << "Sent: " << packet << endl;
        }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_base_node>();
    std::thread reader_thread(&robot_base_node::THREAD__serial_reader, node.get());
    
    std::thread io_thread([node](){
        node->io_context_.run();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();

    reader_thread.join();
    io_thread.join();
    
    return 0;
}
