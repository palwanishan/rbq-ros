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

#include <stdio.h>
#include "Eigen/Dense"
#include "Eigen/Core"

#define MAX_MC      12
#define MAX_JOINT   12
#define MAX_LEG      4
#define MAX_COMMAND_DATA        40


enum COMMAND {
    NO_ACT = 0,
    CAN_CHECK,
    FIND_HOME,
    MOTOR_ONOFF,
    REF_ONOFF,
    SETTING_DQ_ALIGN,
    SETTING_CUR_NULLING,
    SETTING_HOME_ZERO,
    SETTING_MANUAL_HOME,
    SETTING_REQ_FOC_GAIN,
    SETTING_SET_FOC_GAIN,
    SETTING_REQ_POS_GAIN,
    SETTING_SET_POS_GAIN,
    SETTING_DQ_SHIFT,
    SETTING_DQ_MANUAL,
    SETTING_DQ_MANUAL_ROLL,
    SETTING_IMU_RESET,
    SETTING_JOY_CONNECT,
    SETTING_CURRENT_LIMIT,
    SETTING_DGAIN_MODE,

    MOTION_READY = 100,
    MOTION_GROUND,
    MOTION_HOMMING,
    MOTION_JOG,
    CONTROLMODE_JOINT,
    CONTROLMODE_TASK,
    REFMODE_JOINT,
    REFMODE_TASK,
    CONTROL_START,
    ROBOT_CONTROL_START,
    CONTROL_PARA_SET,
    SAVE_DATA,
    SAVE_DATA_START,
    GAMEPAD_DATA,
    MOTION_STAND_UP,
    MOTION_LIFT,

    BASIC_TROT_READY = 200,
    BASIC_TROT_START,
    BASIC_TROT_STOP,
    BASIC_TROT_PARA_UPDATE,

    BASIC_BACK_FLIP,
    FALL_RECOVERY,

    MPC_WALK_READY,
    MPC_GAIT_TRANSITION,
    QP_WALK_READY,
    MPC_PARA_UPDATE,
    SWING_LEG_TEST,
    STAIRS_GO,
    EXT_JOY_ONOFF,

    TO_LOW_HEIGHT = 300,
    TO_MID_HEIGHT,
    TO_HIGH_HEIGHT,

    AUTO_START = 400,
    DISPLAY_START,
    CMD_BASIC_JSON_SAVE,
    CMD_BASIC_JSON_LOAD,
    CMD_MPC_JSON_SAVE,
    CMD_MPC_JSON_LOAD,
    LCM_TEST,

    APP_RESTART = 500,
    COMPUTER_RESTART,
    COMPUTER_SHUTDOWN,
    EMERGENCY_SWITCH,
};



typedef union _STAT_WORD_
{
    struct{
        unsigned    CON_START:1;
        unsigned    READY_POS:1;
        unsigned    GROUND_POS:1;
        unsigned    FORCE_CON:1;
        unsigned    EXT_JOY:1;
        unsigned    IS_STANDING:1;
        unsigned    b:1;
        unsigned    c:1;
    }stat;
    unsigned char B;
}STAT_WORD;

typedef struct _STEP_DATA_
{
    Eigen::Vector3f Fpos_ori0[4], Fpos_ori1[4], Fpos_ori2[4];
    Eigen::Vector3f Fpos0[4], Fpos1[4], Fpos2[4];
    Eigen::Vector3f ZMP_ref[10];
    Eigen::Vector3f ZMP_cur, COM_ref;
}STEP_DATA;

typedef union{
    struct{
        unsigned    FET:1;	 	// FET ON   //
        unsigned    RUN:1;		// Control ON
        unsigned    INIT:1;     // Init Process Passed  //
        unsigned    MOD:1;		// Control Mode
        unsigned    NON_CTR:1;		// Nonius Count err //
        unsigned    BAT:1;      // Low Battery //
        unsigned    CALIB:1;    // Calibration Mode //
        unsigned    MT_ERR:1;      // Reply Status //

        unsigned    JAM:1;		// JAM Error
        unsigned    CUR:1;		// Over Current Error
        unsigned    BIG:1;		// Big Position Error
        unsigned    INP:1;      // Big Input Error
        unsigned    FLT:1;		// FET Driver Fault Error //
        unsigned    TMP:1;      // Temperature Error //
        unsigned    PS1:1;		// Position Limit Error (Lower) ////
        unsigned    PS2:1;		// Position Limit Error (Upper)

        unsigned    rsvd:8;

        unsigned    KP:16;
        unsigned    KD:16;
    }b;
    unsigned char B[7];
}mSTAT;

typedef struct _TASK_INFO_ {
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;
    Eigen::Vector3f force;
    Eigen::Vector4f quat;
    Eigen::Matrix3f rot;
    Eigen::Vector3f rpy;
    Eigen::Vector3f omega;
    Eigen::Matrix3f Jacobian;
    Eigen::Vector3f kp;
    Eigen::Vector3f kd;
} TASK_INFO;

typedef struct _JOINT_INFO_ {
    float pos;
    float vel;
    float acc;
    float torque;
    float kp;
    float kd;
} JOINT_INFO;

typedef struct _ROBOT_INFO_ {
    TASK_INFO   W_body, B_body, Brp_body, Brpy_body;
    TASK_INFO   W_leg[MAX_LEG], B_leg[MAX_LEG], Brp_leg[MAX_LEG], Brpy_leg[MAX_LEG];
    Eigen::Vector4i    contact;
    JOINT_INFO  joint[MAX_JOINT];
} ROBOT_INFO;

typedef struct _MOTOR_INFO_ {
    float pos;
    float vel;
    float acc;
    float cur;
    char  temp;
    mSTAT status;
    bool connect;
} MOTOR_INFO;

typedef struct _IMU_INFO_ {
    Eigen::Vector4f quat;
    Eigen::Vector3f rpy;
    Eigen::Vector3f gyro;
    Eigen::Vector3f acc;
} IMU_INFO;

typedef struct _JOY_INFO_ {
    float L_UD, L_RL, R_UD, R_RL;
    float LT, RT;
    bool BTN[15];
    bool status;
} JOY_INFO;

typedef struct _DISPLAY_INFO_ {
    unsigned char data1;
    unsigned char data2;
} DISPLAY_INFO;

typedef struct _BATTERY_INFO_ {
    float voltage;
    float percent_level;
} BATTERY_INFO;

typedef struct _SENSOR_INFO_ {
    MOTOR_INFO motor[MAX_MC];
    IMU_INFO imu;
    JOY_INFO joy;
    BATTERY_INFO battery;
    DISPLAY_INFO display;
} SENSOR_INFO;

//---------------------Communication Struct---------------------

// Client --> Robot
typedef struct _USER_COMMAND_
{
    int     USER_COMMAND;
    char    USER_PARA_CHAR[MAX_COMMAND_DATA];
    int     USER_PARA_INT[MAX_COMMAND_DATA];
    float   USER_PARA_FLOAT[MAX_COMMAND_DATA];
    double  USER_PARA_DOUBLE[MAX_COMMAND_DATA];
} USER_COMMAND, *pUSER_COMMAND;

// Robot --> Client
typedef struct _ROBOT_STATE_DATA_ {
    ROBOT_INFO   State;
    ROBOT_INFO   ControlRef;
    ROBOT_INFO   CommandRef;
    SENSOR_INFO  Sensor;
    MOTOR_INFO   motor_ref[MAX_MC];
    float        custom_variable[20];
    STAT_WORD    StatusWord;
    STEP_DATA    StepData;
} ROBOT_STATE_DATA, *pROBOT_STATE_DATA;



