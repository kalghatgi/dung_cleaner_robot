#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
#include <iostream>
#include <thread>
///////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "odometry.hpp"
#include <boost/asio.hpp>
#include "nlohmann/json.hpp"

#define LIMIT(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SIGN(x) ((x > 0) ? 1 : -1)

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace boost::asio;
using namespace std;
using json = nlohmann::json;

class robot_base_node : public rclcpp::Node
{
    Odometry wheel_odom;
    tf2::Quaternion wheel_odom_quaternion;
    bool command_received_ = false;
    rclcpp::Time last_command_time_;
    const double WATCHDOG_TIMEOUT = 0.5;

    public:
        // for transferring the parameter value into a variable for use inside the code
        rclcpp::Parameter _velocity_input_topic;
        // for using inside the code, the value held by these parameters
        std::string velocity_input_topic_;

        robot_base_node(): Node("AMR__robot_base__ROS_node")
        {
            this->declare_parameter("velocity_input_topic", "cmd_vel");
            _velocity_input_topic = this->get_parameter("velocity_input_topic");
            velocity_input_topic_ = _velocity_input_topic.as_string();

            cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                velocity_input_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&robot_base_node::cmd_vel_callback, this, _1));

            // create publishers for imu/data_raw and imu/mag
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
        }

    private:
        io_service io;
        serial_port serial(io, "/dev/ttyUSB1");
        const double WHEEL_RADIUS = 0.100; // meters
        const double WHEEL_CIRCUMFERENCE = (2 * M_PI * WHEEL_RADIUS); // meters
        const double WHEEL_SEPERATION = 0.555; // meters
        double NEW_command_linear_X = 0, NEW_command_angular_Z = 0;
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

        void hardware_node_data_receiver_callback(const boost::system::error_code &error, size_t bytes_transferred, streambuf &buffer) // receive any / all data from the hardware
        {
            if (!error)
            {
                istream is(&buffer);
                string receivedPacket;
                getline(is, receivedPacket);
                try
                {
                    json from_hardware_node = json::parse(receivedPacket);
                    cout << "Received JSON: " << from_hardware_node.dump(4) << endl;

                    // Calculate the wheel odometry, and publish to ROS.
                    wheel_odom.setWheelParams(WHEEL_SEPERATION, WHEEL_RADIUS, WHEEL_RADIUS);
                    wheel_odom.updateFromVelocity
                    (
                        (-1.0f * from_hardware_node["rbf"]["wf"]["v"][0] * _dt * WHEEL_CIRCUMFERENCE), // meters per second
                        (from_hardware_node["rbf"]["wf"]["v"][1] * _dt * WHEEL_CIRCUMFERENCE),
                        _dt
                    );
                    double _Position_X = wheel_odom.getX(); // aka cartesian coordinates
                    double _Position_Y = wheel_odom.getY();
                    wheel_odom_quaternion.setRPY(0.0, 0.0, wheel_odom.getHeading());
                    double _LinearVelocity_X = wheel_odom.getLinear();
                    double _AngularVelocity_Z = wheel_odom.getAngular();
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
                    odometry_message.pose.pose.position.x = _Position_X;
                    odometry_message.pose.pose.position.y = _Position_Y;
                    odometry_message.pose.pose.orientation.w = wheel_odom_quaternion.w();
                    odometry_message.pose.pose.orientation.x = wheel_odom_quaternion.x();
                    odometry_message.pose.pose.orientation.y = wheel_odom_quaternion.y();
                    odometry_message.pose.pose.orientation.z = wheel_odom_quaternion.z();
                    odometry_message.twist.twist.linear.x = _LinearVelocity_X;
                    odometry_message.twist.twist.angular.z = _AngularVelocity_Z;
                    wheel_odometry_publisher_->publish(odometry_message);
                    
                    // adjust the IMU data, and publish to ROS
                    auto imu_raw_message = sensor_msgs::msg::Imu();
                    imu_raw_message.header.stamp = _now;
                    imu_raw_message.linear_acceleration.x = (-1.0f * from_hardware_node["rbf"]["imu"]["a"][0]);
                    imu_raw_message.linear_acceleration.y = from_hardware_node["rbf"]["imu"]["a"][1];
                    imu_raw_message.linear_acceleration.z = from_hardware_node["rbf"]["imu"]["a"][2];
                    imu_raw_message.angular_velocity.x = from_hardware_node["rbf"]["imu"]["g"][0];
                    imu_raw_message.angular_velocity.y = from_hardware_node["rbf"]["imu"]["g"][1];
                    imu_raw_message.angular_velocity.z = from_hardware_node["rbf"]["imu"]["g"][2];
                    imu_raw_publisher_->publish(imu_raw_message);
                    auto imu_mag_message = sensor_msgs::msg::MagneticField();
                    imu_mag_message.header.stamp = _now;
                    imu_mag_message.magnetic_field.x = from_hardware_node["rbf"]["imu"]["m"][0];
                    imu_mag_message.magnetic_field.y = from_hardware_node["rbf"]["imu"]["m"][1];
                    imu_mag_message.magnetic_field.z = from_hardware_node["rbf"]["imu"]["m"][2];
                    imu_mag_publisher_->publish(imu_mag_message);
                }
                catch (json::parse_error &pe)
                {
                    cerr << "JSON parse error: " << pe.what() << endl;
                }
            }
            else
            {
                cerr << "UART receive error: " << error.message() << endl;
            }
        }

        void hardware_node_data_sender_callback() // send all data to the hardware
        {
            // Inverse Jacobian for the velocity of individual wheels
            to_hardware_node["rbc"]["wc"]["v"][0] = (-1.0f * (NEW_command_linear_X + (NEW_command_angular_Z * WHEEL_SEPERATION / 2)) / WHEEL_CIRCUMFERENCE); // rotations per second
            to_hardware_node["rbc"]["wc"]["v"][1] = ((NEW_command_linear_X - (NEW_command_angular_Z * WHEEL_SEPERATION / 2)) / WHEEL_CIRCUMFERENCE);
            
            to_hardware_node["rpc"]["bc"]["t"] = 0.0;
            to_hardware_node["rpc"]["cc"]["t"] = 75.0;

            string packet = data.dump() + "\n"; // Convert JSON to string and append newline
            write(serial, buffer(packet)); // Send over serial
            cout << "Sent: " << packet << endl;
        }

};

int main(int argc, char * argv[])
{
    serial.set_option(serial_port_base::baud_rate(460800));
    serial.set_option(serial_port_base::character_size(8));
    serial.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    streambuf rx_buffer;
    async_read_until(serial, rx_buffer, "\n", bind(hardware_node_data_receiver_callback, placeholders::_1, placeholders::_2, ref(rx_buffer)));

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robot_base_node>());
    rclcpp::shutdown();
    return 0;
}
