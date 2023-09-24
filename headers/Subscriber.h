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
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include "RobotApiHandler.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber(const std::shared_ptr<RobotApiHandler> &robotApiHandler) 
        : Node("rbq_subscriber")
        , m_robotApiHandler(robotApiHandler) 
        {
        m_subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
            "rbq/twist", 10, std::bind(&Subscriber::callback_twist, this, _1));

        m_subscription_autoStart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/auto_start", 10, std::bind(&Subscriber::callback_autoStart, this, _1));

        m_subscription_canCheck = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/can_check", 10, std::bind(&Subscriber::callback_canCheck, this, _1));

        m_subscription_findHome = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/find_home", 10, std::bind(&Subscriber::callback_findHome, this, _1));

        m_subscription_dqCheck = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/dq_check", 10, std::bind(&Subscriber::callback_dqCheck, this, _1));

        m_subscription_controlStart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/control_start", 10, std::bind(&Subscriber::callback_controlStart, this, _1));

        m_subscription_motionStaticReady = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_static_ready", 10, std::bind(&Subscriber::callback_motionStaticReady, this, _1));

        m_subscription_motionStaticGround = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_static_ground", 10, std::bind(&Subscriber::callback_motionStaticGround, this, _1));

        m_subscription_motionDynamicReady = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_ready", 10, std::bind(&Subscriber::callback_motionDynamicReady, this, _1));

        m_subscription_motionDynamicSteady = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_steady", 10, std::bind(&Subscriber::callback_motionDynamicSteady, this, _1));

        m_subscription_motionDynamicWalk = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_walk", 10, std::bind(&Subscriber::callback_motionDynamicWalk, this, _1));

        m_subscription_motionDynamicWalkSlow = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_walk_slow", 10, std::bind(&Subscriber::callback_motionDynamicWalkSlow, this, _1));

        m_subscription_motionsDynamicRun = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_run", 10, std::bind(&Subscriber::callback_motionsDynamicRun, this, _1));

        m_subscription_motionDynamicRunFast = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motion_dynamic_run_fast", 10, std::bind(&Subscriber::callback_motionDynamicRunFast, this, _1));

        m_subscription_switchGamepadChannel = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/switch_gamepad_udp_port", 10, std::bind(&Subscriber::callback_switchGamepadChannel, this, _1));

        m_subscription_controlAppRestart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/control_app_restart", 10, std::bind(&Subscriber::callback_controlAppRestart, this, _1));

        m_subscription_controlComputerRestart = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/control_pc_restart", 10, std::bind(&Subscriber::callback_controlComputerRestart, this, _1));

        m_subscription_emergencyOFF = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motor_off", 10, std::bind(&Subscriber::callback_emergencyOFF, this, _1));

        m_subscription_emergencyON = this->create_subscription<std_msgs::msg::Bool>(
            "rbq/motor_on", 10, std::bind(&Subscriber::callback_emergencyON, this, _1));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_subscription_twist;
    void callback_twist(const geometry_msgs::msg::Twist::SharedPtr _twist) const 
    {
        // RCLCPP_INFO(this->get_logger(), "I heard twist.x: '%f'", _twist->linear.x);
        Eigen::Vector3f linear_ = Eigen::Vector3f(_twist->linear.x, _twist->linear.y, _twist->linear.z);
        Eigen::Vector3f angular_ = Eigen::Vector3f(_twist->angular.x, _twist->angular.y, _twist->angular.z);

        if(linear_.x() > 1.0f) {
            linear_.x() = 1;
        }
        else if(linear_.x() < -1.0f) {
            linear_.x() = -1;
        }
        if(linear_.y() > 1.0f) {
            linear_.y() = 1;
        }
        else if(linear_.y() < -1.0f) {
            linear_.y() = -1;
        }
        if(linear_.z() > 1.0f) {
            linear_.z() = 1;
        }
        else if(linear_.z() < -1.0f) {
            linear_.z() = -1;
        }

        if(angular_.x() > 1.0f) {
            angular_.x() = 1;
        }
        else if(angular_.x() < -1.0f) {
            angular_.x() = -1;
        }
        if(angular_.y() > 1.0f) {
            angular_.y() = 1;
        }
        else if(angular_.y() < -1.0f) {
            angular_.y() = -1;
        }
        if(angular_.z() > 1.0f) {
            angular_.z() = 1;
        }
        else if(angular_.z() < -1.0f) {
            angular_.z() = -1;
        }


        uchar lx = (-linear_.y()+1.0)*100; // left stick  X Axis range:0~200
        uchar ly = (-linear_.x()+1.0)*100; // left stick  Y Axis range:0~200
        uchar rx = (-angular_.z()+1.0)*100;// right stick X Axis range:0~200
        uchar ry = (-angular_.y()+1.0)*100;// right stick Y Axis range:0~200
        uchar l2 = 0;
        uchar r2 = 0;
        uchar bt0 = 0;
        uchar bt1 = 0;

        uchar msg[12];

        // header
        msg[0] = 0xFF;
        msg[1] = 0xFE;

        // body
        memcpy(&msg[2], &lx, 1);
        memcpy(&msg[3], &ly, 1);
        memcpy(&msg[4], &rx, 1);
        memcpy(&msg[5], &ry, 1);
        memcpy(&msg[6], &l2, 1);
        memcpy(&msg[7], &r2, 1);
        memcpy(&msg[8], &bt0, 1);
        memcpy(&msg[9], &bt1, 1);

        // tail
        msg[10] = 0x00;
        msg[11] = 0x01;

        if(m_robotApiHandler != nullptr) {
            m_robotApiHandler->setGamepadCommand((const char *)&msg, 12);
        }

    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_autoStart;
    void callback_autoStart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->autoStart();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_canCheck;
    void callback_canCheck(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->canCheck();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_findHome;
    void callback_findHome(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->findHome();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_dqCheck;
    void callback_dqCheck(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->dqCheck();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_controlStart;
    void callback_controlStart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->controlStart();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionStaticReady;
    void callback_motionStaticReady(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionStaticReady();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionStaticGround;
    void callback_motionStaticGround(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionStaticGround();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionDynamicReady;
    void callback_motionDynamicReady(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionDynamicReady();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionDynamicSteady;
    void callback_motionDynamicSteady(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionDynamicSteady();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionDynamicWalk;
    void callback_motionDynamicWalk(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionDynamicWalk();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionDynamicWalkSlow;
    void callback_motionDynamicWalkSlow(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionDynamicWalkSlow();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionsDynamicRun;
    void callback_motionsDynamicRun(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionsDynamicRun();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_motionDynamicRunFast;
    void callback_motionDynamicRunFast(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->motionDynamicRunFast();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_switchGamepadChannel;
    void callback_switchGamepadChannel(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->switchGamepadChannel();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_controlAppRestart;
    void callback_controlAppRestart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->controlAppRestart();
        }
    }    
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_controlComputerRestart;
    void callback_controlComputerRestart(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->controlComputerRestart();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_controlComputerShutdown;
    void callback_controlComputerShutdown(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->controlComputerShutdown();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_emergencyOFF;
    void callback_emergencyOFF(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->emergencyOFF();
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscription_emergencyON;
    void callback_emergencyON(const std_msgs::msg::Bool::SharedPtr _confirm) const {
        if(_confirm) {
            m_robotApiHandler->emergencyON();
        }
    }
    
    std::shared_ptr<RobotApiHandler> m_robotApiHandler = nullptr;
};