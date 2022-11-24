
/*
    Copyright (c) 2022
    ITMO University

    Author:
    Kirill Artemov

    License: MIT
*/

#ifndef IIWA_DRIVER_WRAPPER_H_
#define IIWA_DRIVER_WRAPPER_H_

#include <string>
#include <vector>
#include <mutex>
#include <chrono>

#include <lcm/lcm-cpp.hpp>
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"


#include <ros/ros.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

#include "iiwa_driver_ros_interface/JointPositions.h"
#include "iiwa_driver_ros_interface/JointVelocities.h"
#include "iiwa_driver_ros_interface/JointTorques.h"

using drake::lcmt_iiwa_command;
using drake::lcmt_iiwa_status;

const char *kLcmStatusChannel = "IIWA_STATUS";
const char *kLcmCommandChannel = "IIWA_COMMAND";
const int kNumJoints = 7;


namespace iiwa {

class DriverWrapper {
//    private:
public:
    DriverWrapper();

    std::mutex mtx;

    lcm::LCM lcm;
    lcmt_iiwa_status lcm_status{};
    lcmt_iiwa_command lcm_command{};
  
    double rate; // [Hz]
    int iiwa_dof;
    
    std::string iiwa_world_frame;
    std::string iiwa_base_frame;
    std::string iiwa_ee_frame;

    ros::NodeHandle node;

    sensor_msgs::JointState iiwa_joint_state;

    ros::Publisher iiwa_joint_states_publisher;
    ros::Subscriber position_subscriber;
    ros::Subscriber velocities_subscriber;
    ros::Subscriber torques_subscriber;

   
    DriverWrapper(ros::NodeHandle n);
    virtual ~DriverWrapper();
    void initialize();
    void stop();
    
    void set_rate(double rate);
    
    void handle_joint_state(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const drake::lcmt_iiwa_status *msg);
    
    void joint_states_update();

    void joint_positions_callback(const iiwa_driver_ros_interface::JointPositionsConstPtr& cmd_positions);
    void joint_velocities_callback(const iiwa_driver_ros_interface::JointVelocitiesConstPtr& cmd_velocities);
    void joint_torques_callback(const iiwa_driver_ros_interface::JointTorquesConstPtr& cmd_torques);

};

}

#endif /* IIWA_DRIVER_WRAPPER_H_ */