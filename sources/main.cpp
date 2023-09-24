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

#include "include/RobotApiHandler.h"
#include "headers/Publisher.h"
#include "headers/Subscriber.h"

// std::string robotNetworkAddrIPv4 = "192.168.10.13";
std::string robotNetworkAddrIPv4 = "192.168.0.10";
int robotNetworkPortTCP = 8000;
int robotNetworkPortUDP = 28224;

void spin(const rclcpp::Node::SharedPtr &_node) {
  rclcpp::spin(_node);
}

int main(int argc, char * argv[])
{
  std::shared_ptr<RobotApiHandler> apiHandler = std::make_shared<RobotApiHandler>(
    robotNetworkAddrIPv4, robotNetworkPortTCP, robotNetworkPortUDP
  );

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr publisher = std::make_shared<Publisher>(apiHandler, 50ms);
  std::thread thread_publisher(spin, publisher);
  
  rclcpp::Node::SharedPtr subscriber = std::make_shared<Subscriber>(apiHandler);
  std::thread thread_subscriber(spin, subscriber);

  while(rclcpp::ok()) {
    rclcpp::sleep_for(1000ms);
  }

  rclcpp::shutdown();
  thread_publisher.join();
  thread_subscriber.join();

  return 0;
}
