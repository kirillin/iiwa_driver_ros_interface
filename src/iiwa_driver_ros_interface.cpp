/*
    Copyright (c) 2022
    ITMO University

    Author:
    Kirill Artemov

    License: MIT
*/

#include "iiwa_driver_ros_interface.hpp"

iiwa::DriverWrapper::DriverWrapper() {}

iiwa::DriverWrapper::DriverWrapper(ros::NodeHandle n) : node(n) {
    iiwa_dof = 7;

    n.param("rate", rate, 200.0);
    // n.param("iiwa_world_frame", iiwa_world_frame, "world");
    // n.param("iiwa_base_frame", iiwa_base_frame, "iiwa_base_link");
    // n.param("iiwa_ee_frame", iiwa_ee_frame, "iiwa_ee_link");
}

iiwa::DriverWrapper::~DriverWrapper() {
    this->stop();
}

void iiwa::DriverWrapper::initialize() {
    try {
        // lcm initialize

        if (!lcm.good())
            std::cout << "lcm problem ad lcm.good()\n";

        lcm.subscribe(kLcmStatusChannel, &iiwa::DriverWrapper::handle_joint_state, this);

        // ros initialize
        iiwa_joint_states_publisher = node.advertise<sensor_msgs::JointState>("joint_states", 1);

        position_subscriber = node.subscribe<iiwa_driver_ros_interface::JointPositions>("iiwa_positions_command", 1000, &iiwa::DriverWrapper::joint_positions_callback, this);
        velocities_subscriber = node.subscribe<iiwa_driver_ros_interface::JointVelocities>("iiwa_velocities_command", 1000, &iiwa::DriverWrapper::joint_velocities_callback, this);
        torques_subscriber = node.subscribe<iiwa_driver_ros_interface::JointTorques>("iiwa_torques_command", 1000, &iiwa::DriverWrapper::joint_torques_callback, this);

        ROS_INFO("iiwa is initialized.");

    } catch (std::exception& e) {
        std::string errormsg = e.what();
        ROS_FATAL("%s", errormsg.c_str());
        return;
    }
}

void iiwa::DriverWrapper::stop() {
    // 0. clean memory

    // 1. shutdown all ros items
    iiwa_joint_states_publisher.shutdown();
    position_subscriber.shutdown();
    velocities_subscriber.shutdown();
    torques_subscriber.shutdown();

    // 2. close lcm
}

void iiwa::DriverWrapper::set_rate(double rate) {
    this->rate = rate;
}

void iiwa::DriverWrapper::handle_joint_state(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const drake::lcmt_iiwa_status* msg) {
    mtx.lock();
    lcm_status = *msg;
    mtx.unlock();
}

void iiwa::DriverWrapper::joint_positions_callback(const iiwa_driver_ros_interface::JointPositionsConstPtr& cmd_positions) {
    if (cmd_positions->positions.size() < 1) {
        ROS_WARN("[CALLBACK J. POSITIONS] Invalid command.");
        return;
    }
}

void iiwa::DriverWrapper::joint_velocities_callback(const iiwa_driver_ros_interface::JointVelocitiesConstPtr& cmd_velocities) {
    if (cmd_velocities->velocities.size() < 1) {
        ROS_WARN("[CALLBACK J. VELOCITIES] Invalid command.");
        return;
    }
}

void iiwa::DriverWrapper::joint_torques_callback(const iiwa_driver_ros_interface::JointTorquesConstPtr& cmd_torques) {
    if (cmd_torques->torques.size() < 1) {
        ROS_WARN("[CALLBACK J. TORQUES] Invalid command.");
        return;
    }

    mtx.lock();

    lcm_command.utime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();;
    lcm_command.num_joints = kNumJoints;
    lcm_command.num_torques = kNumJoints;
    lcm_command.joint_position.resize(kNumJoints, 0);
    lcm_command.joint_torque.resize(kNumJoints, 0);
    for (int i = 0; i < kNumJoints; i++) {
        lcm_command.joint_position[i] = lcm_status.joint_position_measured[i];
        lcm_command.joint_torque[i] = cmd_torques->torques[i];
    }

    lcm.publish("IIWA_COMMAND", &lcm_command);

    mtx.unlock();
}


void iiwa::DriverWrapper::joint_states_update() {
    mtx.lock();
    sensor_msgs::JointState js_msg;
    js_msg.header.stamp = ros::Time::now();

    for (int i = 0; i < iiwa_dof; ++i) {
        js_msg.position.push_back(lcm_status.joint_position_measured[i]);
        js_msg.velocity.push_back(lcm_status.joint_velocity_estimated[i]);
        js_msg.effort.push_back(lcm_status.joint_torque_external[i]);
    }
    mtx.unlock();

    iiwa_joint_states_publisher.publish(js_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "iiwa_driver_ros_interface_node");

    ros::NodeHandle n;
    iiwa::DriverWrapper iiwa_driver_wrapper(n);
    
    iiwa_driver_wrapper.initialize();

    double rate_hz;
    n.param("rate", rate_hz, 200.0);

    ros::Rate rate(rate_hz);
    while (n.ok()) {
        ros::spinOnce();
        iiwa_driver_wrapper.joint_states_update();
        rate.sleep();
    }

    iiwa_driver_wrapper.stop();

    return 0;
}