// Copyright 2023 Rainbow Robotics Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rbq_api.h"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher(const std::shared_ptr<ROBOT_STATE_DATA> &_robotStateNative = nullptr,
            const std::chrono::milliseconds &_m_sleepTime = 100ms)
  : Node("rbq_publisher")
  , m_sleepTime(_m_sleepTime)
  , robotStateNative(_robotStateNative)
  {
    m_publisher_imu = this->create_publisher<sensor_msgs::msg::Imu>("rbq/imu", 10);
    m_timer_imu = this->create_wall_timer(
      m_sleepTime, std::bind(&Publisher::publishImu, this));

    m_publisher_robotState = this->create_publisher<sensor_msgs::msg::JointState>("rbq/robotState", 10);
    m_timer_robotState = this->create_wall_timer(
      m_sleepTime, std::bind(&Publisher::publishRobotState, this));
  }

  void setRobotStateNative(const std::shared_ptr<ROBOT_STATE_DATA> &_robotStateNative) {
    robotStateNative = _robotStateNative;
  }

private:
  void publishImu()
  {
    if(robotStateNative == nullptr) {    RCLCPP_ERROR(this->get_logger(), 
        "Shared Data is set to nullptr.");
      return;
    }
    sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.stamp = this->get_clock()->now();

    imu_msg.orientation.x = robotStateNative->Sensor.imu.quat.x(); 
    imu_msg.orientation.y = robotStateNative->Sensor.imu.quat.y(); 
    imu_msg.orientation.z = robotStateNative->Sensor.imu.quat.z(); 
    imu_msg.orientation.w = robotStateNative->Sensor.imu.quat.w(); 

    imu_msg.angular_velocity.x = robotStateNative->Sensor.imu.gyro.x();
    imu_msg.angular_velocity.y = robotStateNative->Sensor.imu.gyro.y();
    imu_msg.angular_velocity.z = robotStateNative->Sensor.imu.gyro.z();

    imu_msg.linear_acceleration.x = robotStateNative->Sensor.imu.acc.x();
    imu_msg.linear_acceleration.y = robotStateNative->Sensor.imu.acc.y();
    imu_msg.linear_acceleration.z = robotStateNative->Sensor.imu.acc.z();

    // RCLCPP_INFO(this->get_logger(), 
    //     "Publishing rbq/imu -> \n\t ori.x: '%f' \n\t ori.y: '%f', \n\t ori.z: '%f'", 
    //     imu_msg.orientation.x,
    //     imu_msg.orientation.y,
    //     imu_msg.orientation.z);
    m_publisher_imu->publish(imu_msg);
  }

  void publishRobotState()
  {
    if(robotStateNative == nullptr) {    RCLCPP_ERROR(this->get_logger(), 
        "Shared Data is set to nullptr.");
      return;
    }
    sensor_msgs::msg::JointState jointState_msg = sensor_msgs::msg::JointState();
    jointState_msg.header.stamp = this->get_clock()->now();
    for(int i=0; i<12; i++)
    {
      jointState_msg.name.push_back(m_jointNames.at(i));
      jointState_msg.position.push_back(robotStateNative->State.joint[i].pos);
      jointState_msg.velocity.push_back(robotStateNative->State.joint[i].vel);
      jointState_msg.effort.push_back(robotStateNative->State.joint[i].torque);
    }

    m_publisher_robotState->publish(jointState_msg);
  }

  std::chrono::milliseconds m_sleepTime = 100ms;

  rclcpp::TimerBase::SharedPtr m_timer_imu;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher_imu;

  rclcpp::TimerBase::SharedPtr m_timer_robotState;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_publisher_robotState;
	std::vector<std::string> m_jointNames{"HRR", "HRP", "HRK", "HLR", "HLP", "HLK", "FRR", "FRP", "FRK", "FLR", "FLP", "FLK"};

  std::shared_ptr<ROBOT_STATE_DATA> robotStateNative;
};

