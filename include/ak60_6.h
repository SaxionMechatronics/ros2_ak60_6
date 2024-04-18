#ifndef _AK60_6_H_
#define _AK60_6_H_

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_srvs/srv/empty.hpp"
#include "ros2_ak60_6/msg/motor_command.hpp"
#include "ros2_ak60_6/msg/motor_reading.hpp"
#include "ros2_ak60_6/srv/set_kp_kd.hpp"
#include "ros2_ak60_6/srv/set_can_id.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

//value Limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -65.0f
#define V_MAX 65.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

#define DEFAULT_STIFFNESS 1.0
#define DEFAULT_DAMPENING 0.5

class ak60_6 : public rclcpp::Node
{
  public:
    ak60_6() : Node("AK60_6"){
      can_motor_pub_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus", 10);
      motor_reading_pub_ = this->create_publisher<ros2_ak60_6::msg::MotorReading>("motor_reading", 10);

      timer_ = this->create_wall_timer(10ms, std::bind(&ak60_6::sendMotorCmd, this));

      can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>("/from_can_bus", 10, std::bind(&ak60_6::unpackCanMsg, this, _1));
      input_command_subscriber_ = this->create_subscription<ros2_ak60_6::msg::MotorCommand>("motor_input", 10, std::bind(&ak60_6::setMotorCommandCallback, this, _1));

      enable_motor_service_ = this->create_service<std_srvs::srv::Empty>("enable_motor", std::bind(&ak60_6::enableMotor, this, _1, _2));
      disable_motor_service_ = this->create_service<std_srvs::srv::Empty>("disable_motor", std::bind(&ak60_6::disableMotor, this, _1, _2));
      zero_motor_service_ = this->create_service<std_srvs::srv::Empty>("zero_motor", std::bind(&ak60_6::zeroMotor, this, _1, _2));
      set_kp_kd_service_ = this->create_service<ros2_ak60_6::srv::SetKpKd>("set_KpKd", std::bind(&ak60_6::setKpKd, this, _1, _2));
      set_can_id_service_ = this->create_service<ros2_ak60_6::srv::SetCanId>("set_can_id", std::bind(&ak60_6::setCanId, this, _1, _2));

      this->declare_parameter("motor_can_id", 1);
      this->declare_parameter("motor_dampening", 1.0);
      this->declare_parameter("motor_stiffness", 0.5);

      can_id_ = this->get_parameter("motor_can_id").as_int();
      kp_in = this->get_parameter("motor_stiffness").as_double(); 
      kd_in = this->get_parameter("motor_dampening").as_double();

      RCLCPP_INFO(this->get_logger(), "Can id of motor %d", can_id_);
      RCLCPP_INFO(this->get_logger(), "Motor_stiffness %f", kp_in);
      RCLCPP_INFO(this->get_logger(), "Kd in %f", kd_in);
    }

    //mathmatical functions
    float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits);
    unsigned int float_to_uint(float x, float x_min, float x_max, int bits);
    float constrain(float x, float min, float max);

    //ROS stuff
    void unpackCanMsg(const can_msgs::msg::Frame::SharedPtr msg);
    void sendMotorCmd();
    void setMotorCommandCallback(const ros2_ak60_6::msg::MotorCommand::SharedPtr msg);

    //services
    void enableMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void disableMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void zeroMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
          std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void setKpKd(const std::shared_ptr<ros2_ak60_6::srv::SetKpKd::Request> request,
        std::shared_ptr<ros2_ak60_6::srv::SetKpKd::Response> response);
    void setCanId(const std::shared_ptr<ros2_ak60_6::srv::SetCanId::Request> request,
        std::shared_ptr<ros2_ak60_6::srv::SetCanId::Response> response);
        

  private:
    float p_in = 0.0;   //position
    float v_in = 0.0;   //velocity
    float kp_in = DEFAULT_STIFFNESS; //stiffness
    float kd_in = DEFAULT_DAMPENING;  //dampening
    float t_in = 0.0;  //torque
    int can_id_ = 0x01;  //Can id

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_motor_pub_;
    rclcpp::Publisher<ros2_ak60_6::msg::MotorReading>::SharedPtr motor_reading_pub_;

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
    rclcpp::Subscription<ros2_ak60_6::msg::MotorCommand>::SharedPtr input_command_subscriber_;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr enable_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr disable_motor_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr zero_motor_service_;
    rclcpp::Service<ros2_ak60_6::srv::SetKpKd>::SharedPtr set_kp_kd_service_;
    rclcpp::Service<ros2_ak60_6::srv::SetCanId>::SharedPtr set_can_id_service_;
};


#endif