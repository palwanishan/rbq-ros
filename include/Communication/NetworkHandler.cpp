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

#include "NetworkHandler.h"

#include <QCoreApplication>


NetworkHandler::NetworkHandler(const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative, 
  const std::string &robotNetworkAddrIPv4, const int &robotNetworkPortTCP, const int &robotNetworkPortUDP) 
    : m_robotStateNative(robotStateNative) 
    , m_robotNetworkAddrIPv4(robotNetworkAddrIPv4)
    , m_robotNetworkPortTCP(robotNetworkPortTCP)
    , m_robotNetworkPortUDP(robotNetworkPortUDP)
{      
  if(m_qAppThread == nullptr) {
    m_qAppThread = new std::thread(&NetworkHandler::qAppThread, this);
  }

  m_udp = std::make_shared<UDP>(m_robotNetworkAddrIPv4, m_robotNetworkPortUDP);
}

void NetworkHandler::qAppThread() 
{
  int argc=0; char* argv[] = { (char*)"RBQ-client" };
  QCoreApplication a(argc, argv);
  m_tcp = std::make_shared<TcpClient>(&a, m_robotStateNative);
  // m_tcp->setHost("192.168.10.13"); m_tcp->setPort(8000);
  m_tcp->setHost(QString::fromStdString(m_robotNetworkAddrIPv4)); m_tcp->setPort(m_robotNetworkPortTCP);
  m_tcp->start();
  a.exec();
  return;
}