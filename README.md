# README #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###
This is a ROS2 package for the AK60_6 motors developed by Saxion Smart

### How do I get set up? ###

* Install the required dependencies
Build ros2_socketcan in your workspace [link](https://github.com/autowarefoundation/ros2_socketcan)

### How to use ###
Set up a connection with your can adapter and read the motor values:
```console
sudo ip link set can0 up type can bitrate 1000000
ros2 launch ros2_socketcan socket_can_bridge.launch.xml
```
Then in a new terminal
```console
ros2 launch ros2_ak60_6 motor_launch.py 
```

### Parameter list ###
**motor_can_id:** set to the can_id of the motor you would like to talk to  
**motor_stiffness:** Set the default stiffness  
**motor_dampening:** Set the default dampening
  
### Service list ###
**/enable_motor:** Enables the motor to start spinning  
**/disable_motor:** Disables the motor  
**/zero_motor:** Sets current position of the motor to 0  
**/set_can_id:** Changes the id of the motor is listening to (This does not change the ID of the motor)  
**/set_KdKd:** Change motor stiffness and dampening

### Topic list: ###
**/motor_input:** This topic can be used to give commands to the motors  
**/motor_reading:** Motor feedback topic 
