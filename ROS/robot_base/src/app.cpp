#include <chrono>
#include <functional>
#include <memory>
#include <math.h>
///////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "odometry.hpp"

#define LIMIT(x, min, max) ((x < min) ? min : (x > max) ? max : x)
#define SIGN(x) ((x > 0) ? 1 : -1)

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class robot_base_node : public rclcpp::Node
{
  Odometry wheel_odom;
  tf2::Quaternion wheel_odom_quaternion;
  public:
    // for transferring the parameter value into a variable for use inside the code
    rclcpp::Parameter _velocity_input_topic;
    // for using inside the code, the value held by these parameters
    std::string velocity_input_topic_;

    robot_base_node(): Node("AMR__robot_base__ROS_node")
    {
      this->declare_parameter("velocity_input_topic", "/cmd_vel");
      _velocity_input_topic = this->get_parameter("velocity_input_topic");
      velocity_input_topic_ = _velocity_input_topic.as_string();

      encoder_raw_subscription_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/amr/encoder_raw", rclcpp::SensorDataQoS(), std::bind(&robot_base_node::encoder_raw_callback, this, _1));

      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        velocity_input_topic_, rclcpp::SystemDefaultsQoS(), std::bind(&robot_base_node::cmd_vel_callback, this, _1));

      wheel_odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom/wheel_encoder", rclcpp::SystemDefaultsQoS());
      wheel_odometry_timer_ = this->create_wall_timer(
        100ms, std::bind(&robot_base_node::wheel_odometry_timer_callback, this));

      wheel_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/amr/wheel_speed", rclcpp::SystemDefaultsQoS());
      wheel_velocity_timer_ = this->create_wall_timer(
        50ms, std::bind(&robot_base_node::wheel_velocity_timer_callback, this));

      current_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/amr/current_speed", rclcpp::SystemDefaultsQoS());
      current_velocity_timer_ = this->create_wall_timer(
        50ms, std::bind(&robot_base_node::current_velocity_timer_callback, this));
    }

  private:
    const double WHEEL_RADIUS = 0.088; // meters
    const double WHEEL_SEPERATION = 0.682; // meters
    const uint16_t ENCODER_PPR = 2000; // Reference: https://robokits.co.in/motors/rhino-planetary-geared-24v-motor/100w-24v-encoder-servo-motor/rhino-servo-24v-60rpm-100w-ig52-extra-heavy-duty-planetary-encoder-servo-motor-160kgcm#:~:text=Quad%20Encoder%20requires-,2000,-Pulses%20Per%20Revolution
    const uint16_t MOTOR_GEAR_RATIO = 47; // Reference: https://robokits.co.in/motors/rhino-planetary-geared-24v-motor/100w-24v-encoder-servo-motor/rhino-servo-24v-60rpm-100w-ig52-extra-heavy-duty-planetary-encoder-servo-motor-160kgcm#:~:text=ratio%20is%201%20%3A-,47,-the%20Optical%20encoder
    const uint16_t ENCODER_TICKS_PER_WHEEL_ROTATION = ENCODER_PPR * MOTOR_GEAR_RATIO;
    const double WHEEL_METERS_PER_TICK = (2 * M_PI * WHEEL_RADIUS) / (ENCODER_PPR * MOTOR_GEAR_RATIO); // meters
    int64_t NEW_encoder_value_LEFT_wheel = 0, NEW_encoder_value_RIGHT_wheel = 0;
    int64_t PREVIOUS_encoder_value_LEFT_wheel = 0, PREVIOUS_encoder_value_RIGHT_wheel = 0;
    double NEW_command_linear_X = 0, NEW_command_angular_Z = 0;
    double CURRENT_velocity_LEFT_wheel = 0, CURRENT_velocity_RIGHT_wheel = 0;
    double NEXT_velocity_LEFT_wheel, NEXT_velocity_RIGHT_wheel;
    float PREVIOUS_velocity_error_LEFT_wheel = 0, PREVIOUS_velocity_error_RIGHT_wheel = 0;
    float Integration_velocity_error_LEFT_wheel = 0, Integration_velocity_error_RIGHT_wheel = 0;
    float Duty_LEFT_Wheel = 0, Duty_RIGHT_Wheel = 0;
    float Percent_duty_cycle_LEFT_wheel = 0, Percent_duty_cycle_RIGHT_wheel = 0;
    rclcpp::Time _encoders_previous_time = this->get_clock()->now();
    rclcpp::Time _imu_previous_time = this->get_clock()->now();
    rclcpp::Time _velocity_previous_time = this->get_clock()->now();
    bool _encoders_first_value = true; // to indicate that this is the first value from encoders

    void encoder_raw_callback(std_msgs::msg::Int64MultiArray::SharedPtr encoder_raw_msg)
    {
      rclcpp::Time _now = this->get_clock()->now();
      double _dt = _now.seconds() - _encoders_previous_time.seconds();
      _encoders_previous_time = _now;
      _dt = 0.05; // manual override on basis of known rate (this is better because the accuracy of time as calculated above is far from expected value)

      NEW_encoder_value_LEFT_wheel = encoder_raw_msg->data[0];
      NEW_encoder_value_RIGHT_wheel = encoder_raw_msg->data[1];
      if(_encoders_first_value)
      {
        PREVIOUS_encoder_value_LEFT_wheel = NEW_encoder_value_LEFT_wheel;
        PREVIOUS_encoder_value_RIGHT_wheel = NEW_encoder_value_RIGHT_wheel;
        _encoders_first_value = false;
      }
      RCLCPP_INFO(this->get_logger(), "encoder_raw_callback: %ld %ld", NEW_encoder_value_LEFT_wheel, NEW_encoder_value_RIGHT_wheel);

      CURRENT_velocity_LEFT_wheel = -(NEW_encoder_value_LEFT_wheel - PREVIOUS_encoder_value_LEFT_wheel) * WHEEL_METERS_PER_TICK / _dt; // meters per second
      CURRENT_velocity_RIGHT_wheel = -(NEW_encoder_value_RIGHT_wheel - PREVIOUS_encoder_value_RIGHT_wheel) * WHEEL_METERS_PER_TICK / _dt;
      // wheel_odom.updateFromVelocity(CURRENT_velocity_LEFT_wheel, CURRENT_velocity_RIGHT_wheel, _now);
      RCLCPP_INFO(this->get_logger(), "encoder_raw_callback: Left Velocity: %.4f m/s, Right Velocity: %.4f m/s", 
              CURRENT_velocity_LEFT_wheel, CURRENT_velocity_RIGHT_wheel);
      wheel_odom.updateFromVelocity(CURRENT_velocity_LEFT_wheel/20, -CURRENT_velocity_RIGHT_wheel/20, _dt);
      

      PREVIOUS_encoder_value_LEFT_wheel = NEW_encoder_value_LEFT_wheel;
      PREVIOUS_encoder_value_RIGHT_wheel = NEW_encoder_value_RIGHT_wheel;
    }
    rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr encoder_raw_subscription_;

    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg)
    {
      NEW_command_linear_X = cmd_vel_msg->linear.x;
      NEW_command_angular_Z = cmd_vel_msg->angular.z;
      RCLCPP_INFO(this->get_logger(), "cmd_vel_callback: Lx:%.2f Az:%.2f", NEW_command_linear_X, NEW_command_angular_Z);
    }
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    void wheel_odometry_timer_callback()
    {
      double _Position_X = wheel_odom.getX(); // aka cartesian coordinates
      double _Position_Y = wheel_odom.getY();
      wheel_odom_quaternion.setRPY(0.0, 0.0, wheel_odom.getHeading());
      double _LinearVelocity_X = wheel_odom.getLinear();
      double _AngularVelocity_Z = wheel_odom.getAngular();

      RCLCPP_INFO(this->get_logger(), "wheel_odometry_timer_callback: Px:%2.2lf Py:%2.2lf Lx:%2.2lf Az:%2.2lf", _Position_X, _Position_Y, _LinearVelocity_X, _AngularVelocity_Z);

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
    }
    rclcpp::TimerBase::SharedPtr wheel_odometry_timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odometry_publisher_;

    void calculate_CommandPercentDutyCycle()
    {
      float CURRENT_velocity_error_LEFT_wheel, Derivative_velocity_error_LEFT_wheel;
      float CURRENT_velocity_error_RIGHT_wheel, Derivative_velocity_error_RIGHT_wheel;

      rclcpp::Time _now = this->get_clock()->now();
      float _dt = _now.seconds() - _velocity_previous_time.seconds();
      _velocity_previous_time = _now;
      _dt = 0.05; // manual override on basis of known rate (this is better because the accuracy of time as calculated above is far from expected value)

      // Inverse Jacobian: Velocity of individual wheels, calculated from received velocity command
      NEXT_velocity_LEFT_wheel = (NEW_command_linear_X - (NEW_command_angular_Z * WHEEL_SEPERATION / 2)); // meters per second
      NEXT_velocity_RIGHT_wheel = -(NEW_command_linear_X + (NEW_command_angular_Z * WHEEL_SEPERATION / 2)); // '-ve' sign because this one has to spin reverse inorder to move the robot along a direction.
      RCLCPP_INFO(this->get_logger(), "calculate_CommandPercentDutyCycle: Next Left Velocity: %.4f, Next Right Velocity: %.4f",
              NEXT_velocity_LEFT_wheel, NEXT_velocity_RIGHT_wheel);

      /* **************************************************** LEFT wheel **************************************************** */
      CURRENT_velocity_error_LEFT_wheel = NEXT_velocity_LEFT_wheel - CURRENT_velocity_LEFT_wheel;
      Integration_velocity_error_LEFT_wheel += (CURRENT_velocity_error_LEFT_wheel * _dt);
      Derivative_velocity_error_LEFT_wheel = (CURRENT_velocity_error_LEFT_wheel - PREVIOUS_velocity_error_LEFT_wheel) / _dt;
      PREVIOUS_velocity_error_LEFT_wheel = CURRENT_velocity_error_LEFT_wheel;
      /* **************************************************** RIGHT wheel **************************************************** */
      CURRENT_velocity_error_RIGHT_wheel = NEXT_velocity_RIGHT_wheel - CURRENT_velocity_RIGHT_wheel;
      Integration_velocity_error_RIGHT_wheel += (CURRENT_velocity_error_RIGHT_wheel * _dt);
      Derivative_velocity_error_RIGHT_wheel = (CURRENT_velocity_error_RIGHT_wheel - PREVIOUS_velocity_error_RIGHT_wheel) / _dt;
      PREVIOUS_velocity_error_RIGHT_wheel = CURRENT_velocity_error_RIGHT_wheel;

      /* ********** P controller ********** */
      // const float Kp = 0.015;
      // Duty_LEFT_Wheel = Kp * CURRENT_velocity_error_LEFT_wheel;
      // Duty_RIGHT_Wheel = Kp * CURRENT_velocity_error_RIGHT_wheel;
      /* ********** PD controller ********** */
      // const float Kp = 0.015, Kd = 0.000125;
      // Duty_LEFT_Wheel = Kp * CURRENT_velocity_error_LEFT_wheel + Kd * Derivative_velocity_error_LEFT_wheel;
      // Duty_RIGHT_Wheel = Kp * CURRENT_velocity_error_RIGHT_wheel + Kd * Derivative_velocity_error_RIGHT_wheel;
      /* ********** PID controller ********** */
      const float Kp = 0.025, Kd = 0.000125, Ki = 0.00005;
      Duty_LEFT_Wheel += Kp * CURRENT_velocity_error_LEFT_wheel + Ki * Integration_velocity_error_LEFT_wheel + Kd * Derivative_velocity_error_LEFT_wheel;
      Duty_RIGHT_Wheel += Kp * CURRENT_velocity_error_RIGHT_wheel + Ki * Integration_velocity_error_RIGHT_wheel + Kd * Derivative_velocity_error_RIGHT_wheel;
      /* ********** %Duty calculation ********** */
      Percent_duty_cycle_LEFT_wheel = LIMIT((Duty_LEFT_Wheel*100), -100, 100);
      Percent_duty_cycle_RIGHT_wheel = LIMIT((-Duty_RIGHT_Wheel*100), -100, 100); // '-ve' sign for right wheel because it has to spin in opposite direction
      RCLCPP_INFO(this->get_logger(), "calculate_CommandPercentDutyCycle: Left Duty Cycle: %.2f%%, Right Duty Cycle: %.2f%%",
              Percent_duty_cycle_LEFT_wheel, Percent_duty_cycle_RIGHT_wheel);

      // RCLCPP_INFO(this->get_logger(), "e_L:%.5f  e_R:%.5f\td_L:%.1f  d_R:%.1f", CURRENT_velocity_error_LEFT_wheel, CURRENT_velocity_error_RIGHT_wheel, Percent_duty_cycle_LEFT_wheel, Percent_duty_cycle_RIGHT_wheel);
    }
    void wheel_velocity_timer_callback()
    {
      calculate_CommandPercentDutyCycle();
      RCLCPP_INFO(this->get_logger(), "wheel_velocity_timer_callback: L:%2.2lf R:%2.2lf", Percent_duty_cycle_LEFT_wheel, Percent_duty_cycle_RIGHT_wheel);

      rclcpp::Time _now = this->get_clock()->now();
      auto velocity_message = sensor_msgs::msg::JointState();
      velocity_message.header.stamp = _now;
      velocity_message.effort.push_back((double)Percent_duty_cycle_LEFT_wheel);
      velocity_message.effort.push_back((double)Percent_duty_cycle_RIGHT_wheel);
      
      wheel_velocity_publisher_->publish(velocity_message);
    }
    rclcpp::TimerBase::SharedPtr wheel_velocity_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_velocity_publisher_;

    void current_velocity_timer_callback()
    {
      rclcpp::Time _now = this->get_clock()->now();
      auto velocity_message = sensor_msgs::msg::JointState();
      velocity_message.header.stamp = _now;
      velocity_message.velocity.push_back((double)CURRENT_velocity_LEFT_wheel);
      velocity_message.velocity.push_back((double)CURRENT_velocity_RIGHT_wheel);
      
      RCLCPP_INFO(this->get_logger(), "current_velocity_timer_callback: Left Wheel Velocity: %.4f, Right Wheel Velocity: %.4f",
              CURRENT_velocity_LEFT_wheel, CURRENT_velocity_RIGHT_wheel);
      
      current_velocity_publisher_->publish(velocity_message);
    }
    rclcpp::TimerBase::SharedPtr current_velocity_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr current_velocity_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot_base_node>());
  rclcpp::shutdown();
  return 0;
}
