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

using namespace std::chrono_literals;
using std::placeholders::_1;

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("rbq_subscriber") {
        // m_subscription_twist = this->create_subscription<geometry_msgs::msg::Twist>(
        //     "rbq/twist", 10, std::bind(&Subscriber::callback_twist, this, _1));
    }

private:
    void callback_twist(const geometry_msgs::msg::Twist _twist) const {
        RCLCPP_INFO(this->get_logger(), "I heard twist.x: '%f'", _twist.linear.x);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr 
        m_subscription_twist;
};