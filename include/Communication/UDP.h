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

#include <arpa/inet.h>
#include <chrono>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <vector>

#include <QBuffer>

#include "Log.h"

#ifdef __APPLE__
#define MSG_CONFIRM 0
#endif

class PacketHandler{};

class UDP
{
public:
    UDP(const std::string &robotNetworkAddrIPv4, const int &robotNetworkPortUDP) 
        : m_robotNetworkAddrIPv4(robotNetworkAddrIPv4)
        , m_robotNetworkPortUDP(robotNetworkPortUDP)
    { 
        // SetupClientSocket("127.0.0.1", 28224, false);
        SetupClientSocket(m_robotNetworkAddrIPv4, m_robotNetworkPortUDP, false);
    }
    ~UDP() { 
        DeleteSocket(); 
    }

private:
    std::string m_robotNetworkAddrIPv4 = "192.168.0.10";
    int m_robotNetworkPortUDP = 28224;

    int m_sockfd;
    struct sockaddr_in m_servaddr;
    bool m_isSocketSetup = false;
    bool m_flagRecieve = false;
    std::thread *m_threadClient = nullptr;
    const int m_packetLength = 4096;

public:

    void DeleteSocket() {
        m_flagRecieve = false;
        m_isSocketSetup = false;
        close(m_sockfd);
        if (m_threadClient != nullptr) {
            m_threadClient->join();
            m_threadClient = nullptr;
        }
    }

    bool SetupClientSocket(const std::string &_ipAddr = "192.168.0.10", 
                            const int &_port = 28224, const bool &_isNeedReceiveThread = false) {
        if ((m_sockfd = ::socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("udp socket creation failed\n");
            m_isSocketSetup = false;
        } 
        else {
            memset(&m_servaddr, 0, sizeof(m_servaddr));

            m_servaddr.sin_family = AF_INET;
            m_servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            m_servaddr.sin_port = htons(_port);

            inet_pton(AF_INET, _ipAddr.data(), &(m_servaddr.sin_addr));
            // std::string str = "127.0.0.1";
            // inet_pton(AF_INET, str.data(), &(m_servaddr.sin_addr));

            // if (bind(m_sockfd, (struct sockaddr *) &m_servaddr, sizeof(m_servaddr)) == -1) {
            //     printf("udp socket bind failed\n");
            //     return false;
            // }
            m_isSocketSetup = true;
            printf("udp socket creation success\n");
        }
        if (m_isSocketSetup && m_threadClient == nullptr && _isNeedReceiveThread) {
            m_threadClient = new std::thread(&UDP::receiveThread, this);
            m_flagRecieve = true;
        }
        return m_isSocketSetup;
    }

    void sendPacket(const QByteArray &msg) {        
        if(m_isSocketSetup) {
            // qDebug() << "UDP::sendPacket() : " << msg;
            // Log::PrintBytes(std::cout, "Gamepad: ", msg.data(), 12, true);
            sendto(m_sockfd, msg, msg.length(), MSG_CONFIRM, (const struct sockaddr*)&m_servaddr, sizeof(m_servaddr));
        }
    }


private:
    bool CheckIfClientIsRegistered(const sockaddr_in &_clint_addr) {
        char *s = inet_ntoa(_clint_addr.sin_addr);
        printf("IP address: %s\n", s);
        return true;
    }    
    
    void receiveThread() {
        Log::LogMessage("UDP::receiveThread started!!!");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (m_isSocketSetup) {
            if (m_flagRecieve) {
                timeval timeout;
                timeout.tv_sec = 1;
                timeout.tv_usec = 0;

                fd_set fds;
                FD_ZERO(&fds);
                FD_SET(m_sockfd, &fds);
                int res = ::select(m_sockfd + 1, &fds, 0, 0, &timeout);
                if (res > 0) {
                    char packet[4096];
                    int len = recvfrom(m_sockfd, packet, 4096, MSG_WAITALL, nullptr, nullptr);
                    if (len > 0) {
                        // packet_handler.HandleRawPacket(packet, len);
                        continue;
                    }
                } 
                else {
                    // NO SIGNAL IN LAST 1 SECOND
                    Log::LogMessage("NO SIGNAL");
                }
            } else {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        Log::LogMessage("UDPReceiveThread stopped!!!");
    }

};