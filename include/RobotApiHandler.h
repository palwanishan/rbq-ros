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

#include "rbq_api.h"
#include "Communication/NetworkHandler.h"

class RobotApiHandler
{
public:
    RobotApiHandler(const std::string &robotNetworkAddrIPv4,
        const int &robotNetworkPortTCP,
        const int &robotNetworkPortUDP);

    std::shared_ptr<ROBOT_STATE_DATA> robotStateNative = nullptr;

public:
    void setGamepadCommand(const char *msg, const int &msgSize);

    void autoStart();

    void canCheck();

    void findHome();

    void dqCheck();

    void controlStart();

    void motionStaticReady();

    void motionStaticGround();

    void motionDynamicReady();

    void motionDynamicSteady();

    void motionDynamicWalk();

    void motionDynamicWalkSlow();

    void motionsDynamicRun();

    void motionDynamicRunFast();

    void switchGamepadChannel();

    void controlAppRestart();

    void controlComputerRestart();

    void controlComputerShutdown();

    void emergencyOFF();

    void emergencyON();

    void dqAlign();

    void currentNull();

    inline void setUserCommand(const USER_COMMAND &usrCmd);

private:
    std::shared_ptr<NetworkHandler> m_networkHandler = nullptr;

    bool m_isRobotInitSetupDone = false;
    bool m_isRobotActive = false;
    bool m_isRobotCheckSuccess = false;
    bool m_isFindHomeSuccess = false;
    bool m_isDqCheckSuccess = false;
    bool m_isRobotControlStartSuccess = false;

    std::string m_robotNetworkAddrIPv4 = "192.168.0.10";
    int m_robotNetworkPortTCP = 8000;
    int m_robotNetworkPortUDP = 28224;

};


RobotApiHandler::RobotApiHandler(const std::string &robotNetworkAddrIPv4,
        const int &robotNetworkPortTCP,
        const int &robotNetworkPortUDP) 
    : m_robotNetworkAddrIPv4(robotNetworkAddrIPv4)
    , m_robotNetworkPortTCP(robotNetworkPortTCP)
    , m_robotNetworkPortUDP(robotNetworkPortUDP)
{
    robotStateNative = std::make_shared<ROBOT_STATE_DATA>();
    m_networkHandler = std::make_shared<NetworkHandler>(robotStateNative, 
        m_robotNetworkAddrIPv4, m_robotNetworkPortTCP, m_robotNetworkPortUDP);
    

}

void RobotApiHandler::setGamepadCommand(const char *msg, const int &msgSize) 
{
    m_networkHandler->sendMsgUDP(msg, msgSize);
}

void RobotApiHandler::autoStart()
{
    qDebug() << "RobotApiHandler::autoStart()";
    USER_COMMAND cmd;
    int mode = 1; // Ground pose : 1,
    cmd.USER_COMMAND = AUTO_START;
    cmd.USER_PARA_CHAR[0] = 12;
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::canCheck()
{
    qDebug() << "RobotApiHandler::canCheck()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = CAN_CHECK;
    cmd.USER_PARA_CHAR[0] = 12;
    setUserCommand(cmd);
}

void RobotApiHandler::findHome()
{
    qDebug() << "RobotApiHandler::findHome()";
    USER_COMMAND cmd;
    int mode = 1; // Ground pose : 1,
    // int mode = 5; // sitDown_kneeTouchGround pose : 5,
    cmd.USER_COMMAND = FIND_HOME;
    cmd.USER_PARA_CHAR[0] = 12;
    cmd.USER_PARA_CHAR[1] = mode;
    setUserCommand(cmd);
}

void RobotApiHandler::dqCheck()
{
    qDebug() << "RobotApiHandler::dqCheck()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = SETTING_DQ_MANUAL_ROLL;
    setUserCommand(cmd);
}

void RobotApiHandler::controlStart()
{
    qDebug() << "RobotApiHandler::controlStart()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = ROBOT_CONTROL_START;
    setUserCommand(cmd);
}

void RobotApiHandler::motionStaticReady()
{
    qDebug() << "RobotApiHandler::motionStaticReady()";
   USER_COMMAND cmd;
   cmd.USER_COMMAND = MOTION_READY;
   setUserCommand(cmd);
}

void RobotApiHandler::motionStaticGround()
{
    qDebug() << "RobotApiHandler::motionStaticGround()";
   USER_COMMAND cmd;
   cmd.USER_COMMAND = MOTION_GROUND;
   setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicReady()
{
    qDebug() << "RobotApiHandler::motionDynamicReady()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_WALK_READY;
    cmd.USER_PARA_FLOAT[0] = 0;   // Additional Robot Payload Weight
    cmd.USER_PARA_FLOAT[1] = 0;   // Additional Robot Payload COM X axis w.r.t body center
    cmd.USER_PARA_FLOAT[2] = 0;   // Additional Robot Payload COM Y axis w.r.t body center
    cmd.USER_PARA_FLOAT[3] = 0;   // Additional Robot Payload COM Z axis w.r.t body center

    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicSteady()
{
    qDebug() << "RobotApiHandler::motionDynamicSteady()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = 2; // to Stand
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicWalk()
{
    qDebug() << "RobotApiHandler::motionDynamicWalk()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = 4; //TO Trot
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicWalkSlow()
{
    qDebug() << "RobotApiHandler::motionDynamicWalkSlow()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = 12; // To Trot static
    setUserCommand(cmd);
}

void RobotApiHandler::motionsDynamicRun()
{
    qDebug() << "RobotApiHandler::motionsDynamicRun()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = 6; //To RUN
    setUserCommand(cmd);
}

void RobotApiHandler::motionDynamicRunFast()
{
    qDebug() << "RobotApiHandler::motionDynamicRunFast()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = MPC_GAIT_TRANSITION;
    cmd.USER_PARA_CHAR[0] = 11; //To RUN FAST
    setUserCommand(cmd);
}

void RobotApiHandler::switchGamepadChannel()
{
    qDebug() << "RobotApiHandler::switchGamepadChannel()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = EXT_JOY_ONOFF;
    setUserCommand(cmd);
}

void RobotApiHandler::controlAppRestart()
{
    qDebug() << "RobotApiHandler::controlAppRestart()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = APP_RESTART;
    cmd.USER_PARA_CHAR[0] = 1; // CONFIRM
    setUserCommand(cmd);
}

void RobotApiHandler::controlComputerRestart()
{
    qDebug() << "RobotApiHandler::controlComputerRestart()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = COMPUTER_RESTART;
    cmd.USER_PARA_CHAR[0] = 1; // CONFIRM
    setUserCommand(cmd);
}

void RobotApiHandler::controlComputerShutdown()
{
    qDebug() << "RobotApiHandler::controlComputerShutdown()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = COMPUTER_SHUTDOWN;
    cmd.USER_PARA_CHAR[0] = 1; // CONFIRM

    setUserCommand(cmd);
}

void RobotApiHandler::emergencyOFF()
{
    qDebug() << "RobotApiHandler::emergencyOFF()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = EMERGENCY_SWITCH;
    cmd.USER_PARA_CHAR[0] = 101; // CONFIRM OFF
    setUserCommand(cmd);
}

void RobotApiHandler::emergencyON()
{
    qDebug() << "RobotApiHandler::emergencyON()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = EMERGENCY_SWITCH;
    cmd.USER_PARA_CHAR[0] = 102; // CONFIRM ON
    setUserCommand(cmd);
}

void RobotApiHandler::dqAlign()
{
    qDebug() << "RobotApiHandler::dqAlign()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = SETTING_DQ_ALIGN;
    cmd.USER_PARA_CHAR[0] = 12; // All joints
    setUserCommand(cmd);
}

void RobotApiHandler::currentNull()
{
    qDebug() << "RobotApiHandler::currentNull()";
    USER_COMMAND cmd;
    cmd.USER_COMMAND = SETTING_CUR_NULLING;
    cmd.USER_PARA_CHAR[0] = 12; // All joints
    setUserCommand(cmd);
}

void RobotApiHandler::setUserCommand(const USER_COMMAND &usrCmd)
{
    qDebug() << "RobotApiHandler::setUserCommand()";
    if (m_networkHandler != nullptr) {
        m_networkHandler->sendUserCommand(usrCmd);
    }
}