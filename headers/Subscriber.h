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
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rbq_api.h"
#include "Communication/UDP.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("rbq_subscriber") {
        m_subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
            "rbq/twist", 10, std::bind(&Subscriber::callback_twist, this, _1));
        m_udp = std::make_shared<UDP>();
    }

private:
    void callback_twist(const geometry_msgs::msg::Twist::SharedPtr _twist) const {
        RCLCPP_INFO(this->get_logger(), "I heard twist.x: '%f'", _twist->linear.x);

        uchar msg[12];

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

        if(m_udp != nullptr) {
            QByteArray msgBuffer = QByteArray::fromRawData((const char *) (&msg), 12);
            m_udp->sendPacket(msgBuffer);
        }

    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr 
        m_subscription_twist;

    std::shared_ptr<UDP> m_udp = nullptr;
};