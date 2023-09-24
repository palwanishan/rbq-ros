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

#include "TcpClient.h"
#include "UDP.h"
#include "../rbq_api.h"

class NetworkHandler {

public:
    NetworkHandler(const std::shared_ptr<ROBOT_STATE_DATA> &robotStateNative ,
        const std::string &robotNetworkAddrIPv4,
        const int &robotNetworkPortTCP,
        const int &robotNetworkPortUDP
    );

    void sendMsgUDP(const char *msg, const int &msgSize) {        
        if(m_udp != nullptr) {
            QByteArray msgBuffer = QByteArray::fromRawData(msg, msgSize);
            m_udp->sendPacket(msgBuffer);
        }
    }

    void sendMsgTCP(const uchar msg[], const int &msgSize) {        
        QByteArray sendData = QByteArray::fromRawData((const char *) (&msg), msgSize);
        m_tcp->sendMsg(sendData);        
    }

    void sendUserCommand(const USER_COMMAND &userCmd) {
        qDebug() << "NetworkHandler::sendUserCommand() ";
        QByteArray sendData = QByteArray::fromRawData((const char *) (&userCmd),
                                                      sizeof(USER_COMMAND));
        m_tcp->sendMsg(sendData);
    }

    void setTCP(const std::shared_ptr<TcpClient> &tcp) {
        m_tcp = tcp;
    }

private:
    void qAppThread();


private:
    std::shared_ptr<ROBOT_STATE_DATA> m_robotStateNative = nullptr;
    std::shared_ptr<TcpClient> m_tcp = nullptr;
    std::shared_ptr<UDP> m_udp = nullptr;

    std::string m_robotNetworkAddrIPv4 = "192.168.0.10";
    int m_robotNetworkPortTCP = 8000;
    int m_robotNetworkPortUDP = 28224;

    std::thread *m_qAppThread = nullptr;

};