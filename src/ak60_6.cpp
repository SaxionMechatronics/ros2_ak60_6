#include "ak60_6.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak60_6>());
  rclcpp::shutdown();

  return 0;
}

void ak60_6::sendMotorCmd(){
  can_msgs::msg::Frame can_msg;
  // 0: position[15-8]]
  // 1: position[[7-0]]
  // 2: velocity[[11-4]
  // 3: velocity[[3-0], kp[11-8]
  // 4: [kp[7-0]]
  // 5: [kd[11-4]
  // 6: [kd[3-4], torque[11-8]
  // 7: [torque[7-0]]

  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);  

  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);  
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  can_msg.header.stamp = this->now();
  can_msg.id = can_id_;
  can_msg.dlc = 8;

  can_msg.data[0] = p_int >> 8;
  can_msg.data[1] = p_int & 0xFF;
  can_msg.data[2] = v_int >> 4;
  can_msg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  can_msg.data[4] = kp_int & 0xFF;
  can_msg.data[5] = kd_int >> 4;
  can_msg.data[6] = ((kd_int & 0xF) << 4) | (t_int >>8);
  can_msg.data[7] = t_int & 0xFF;
  
  can_motor_pub_->publish(can_msg);
}


void ak60_6::unpackCanMsg(const can_msgs::msg::Frame::SharedPtr msg){
  if(msg->is_error){
    RCLCPP_INFO(this->get_logger(), "MESSAGE HAS AN ERROR");
  }

  if(msg->data[0] == can_id_){
    ros2_ak60_6::msg::MotorReading motor_reading;

    unsigned int p_int = (msg->data[1] << 8 | msg->data[2]);
    unsigned int v_int = (msg->data[3] << 4 | (msg->data[4] >> 4));
    unsigned int i_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];

    motor_reading.id = msg->data[0];
    motor_reading.position = uint_to_float(p_int, P_MIN, P_MAX, 16);
    motor_reading.velocity = uint_to_float(v_int, V_MIN, V_MAX, 12);
    motor_reading.torque = uint_to_float(i_int, -T_MAX, T_MAX, 12);

    motor_reading_pub_->publish(motor_reading);
  }
}

void ak60_6::enableMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response){
  can_msgs::msg::Frame can_msg ;
  (void) response;
  (void) request; //prevents warning during compilation

  can_msg.header.stamp = this->now();
  can_msg.id = can_id_;
  can_msg.dlc = 8;

  can_msg.data[0] = 0xFF;
  can_msg.data[1] = 0xFF;
  can_msg.data[2] = 0xFF;
  can_msg.data[3] = 0xFF;
  can_msg.data[4] = 0xFF;
  can_msg.data[5] = 0xFF;
  can_msg.data[6] = 0xFF;
  can_msg.data[7] = 0xFC;

  RCLCPP_INFO(this->get_logger(),"Motor %d enabled", can_id_);
  can_motor_pub_->publish(can_msg);
}

void ak60_6::disableMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response){
  can_msgs::msg::Frame can_msg ;
  (void) response;
  (void) request; //prevents warning during compilation

  can_msg.header.stamp = this->now();
  can_msg.id = can_id_;
  can_msg.dlc = 8;

  can_msg.data[0] = 0xFF;
  can_msg.data[1] = 0xFF;
  can_msg.data[2] = 0xFF;
  can_msg.data[3] = 0xFF;
  can_msg.data[4] = 0xFF;
  can_msg.data[5] = 0xFF;
  can_msg.data[6] = 0xFF;
  can_msg.data[7] = 0xFD;

  RCLCPP_INFO(this->get_logger(),"Motor %d disabled", can_id_);
  can_motor_pub_->publish(can_msg);
}

void ak60_6::zeroMotor(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response){
  can_msgs::msg::Frame can_msg ;
  (void) response;
  (void) request; //prevents warning during compilation

  can_msg.header.stamp = this->now();
  can_msg.id = can_id_;
  can_msg.dlc = 8;

  can_msg.data[0] = 0xFF;
  can_msg.data[1] = 0xFF;
  can_msg.data[2] = 0xFF;
  can_msg.data[3] = 0xFF;
  can_msg.data[4] = 0xFF;
  can_msg.data[5] = 0xFF;
  can_msg.data[6] = 0xFF;
  can_msg.data[7] = 0xFE;

  RCLCPP_INFO(this->get_logger(),"Motor %d zero set", can_id_);
  can_motor_pub_->publish(can_msg);
}

void ak60_6::setCanId(const std::shared_ptr<ros2_ak60_6::srv::SetCanId::Request> request,
        std::shared_ptr<ros2_ak60_6::srv::SetCanId::Response> response){
  (void) response; //prevents warning during compilation
  can_id_ = request->can_id;
}

void ak60_6::setKpKd(const std::shared_ptr<ros2_ak60_6::srv::SetKpKd::Request> request,
        std::shared_ptr<ros2_ak60_6::srv::SetKpKd::Response> response){
  (void) response; //prevents warning during compilation
  kp_in = request->kp;
  kd_in = request->kd;
}

void ak60_6::setMotorCommandCallback(const ros2_ak60_6::msg::MotorCommand::SharedPtr msg)
{
  p_in = msg->position;
  v_in = msg->velocity;
  t_in = msg->torque;
}

float ak60_6::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if(bits == 12){
    pgg = ((float)x_int)*span/4095.0 + offset;
  }
  if(bits == 16){
    pgg = ((float)x_int)*span/65535.0 + offset;
  }
  return pgg;
}

unsigned int ak60_6::float_to_uint(float x, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;

  if (bits == 12){
    pgg = (unsigned int) ((x-offset)*4095.0/span);
  } 
  if (bits == 16){
    pgg = (unsigned int) ((x - offset)*65535.0/span);
  }
  return pgg;
}

float ak60_6::constrain(float x, float min, float max) {
  if(x < min) {
    return min;
  }
  else if(max < x) {
    return max;
  }
  else
    return x;
}
