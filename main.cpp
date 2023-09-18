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

#include "headers/rbq_api.h"
#include "headers/Publisher.h"
#include "headers/Subscriber.h"
#include "headers/TcpClient.h"

#include <QCoreApplication>

void spin(const rclcpp::Node::SharedPtr &_node) {
  rclcpp::spin(_node);
}

void qAppThread(const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative, int argc, char * argv[]) {
    QCoreApplication a(argc, argv);
    a.setApplicationName("RBQ-client");

    TcpClient* client = new TcpClient(&a, robotStateNative);
    // client->setHost("192.168.0.10"); client->setPort(8000);
    client->setHost("192.168.10.13"); client->setPort(8000);
    client->start();
    
    a.exec();
}

int main(int argc, char * argv[])
{
  std::shared_ptr<ROBOT_STATE_DATA> robotStateNative = std::make_shared<ROBOT_STATE_DATA>();

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr publisher = std::make_shared<Publisher>(robotStateNative, 50ms);
  std::thread thread_publisher(spin, publisher);
  
  rclcpp::Node::SharedPtr subscriber = std::make_shared<Subscriber>();
  std::thread thread_subscriber(spin, subscriber);

  std::thread thread_qApp(qAppThread, robotStateNative, argc, argv);

  while(rclcpp::ok()) {
    rclcpp::sleep_for(1000ms);
  }

  rclcpp::shutdown();
  thread_publisher.join();
  thread_subscriber.join();

  return 0;
}
