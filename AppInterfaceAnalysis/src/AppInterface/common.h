#ifndef COMMON_H_
#define COMMON_H_

#include <std_msgs/String.h>
#include <ros/ros.h>
#include "commonType/APP_LLM.h"
#include "commonType/RobotControl.h"
#include "commonType/APP_LLM_Algo.h"
#include "APP.h"
#include "InterfaceAnalysis.h"
#include "ROSControl.h"


/// @brief 大模型检测状态结果，成功，失败，超时...
enum LLM_State{
    LLM_Algo_SUCCESS = 0,
    LLM_Algo_FAILED = 1,
    LLM_Algo_TIMEOUT = 2,
};

enum APP_LLM_REQUEST
{
    APP_LLM_REQUEST_Reset = 0,
    APP_LLM_REQUEST_TranceMessage = 1,
    APP_LLM_REQUEST_FirstConfirm = 2,
    APP_LLM_REQUEST_SecondConfirm = 3,
};

struct LLM_str {
    LLM_State state; 
    uint8_t ActionType;
    // 0代表指定物品，算法自动规划路线，抓取，识别顺序，读取TargetObjectName和TargetObjectID
    // 1代表移动到TargetPos中的位置，读取TargetPosName和TargetPosID
    // 2代表检测视野范围内的TargetObject并抓取，读取TargetObjectName和TargetObjectID
    // 3代表前往TargetPos中的位置，并放下物品，读取TargetPosName和TargetPosID
    // 4代表前往TargetPos中的位置，并抓取TargetObject的物体，读取TargetPosName和TargetPosID、TargetObjectName和TargetObjectID
    // 5代表无指令的对话，只需要读取LLM_response即可
    std::string TargetObjectName;
    uint8_t TargetObjectID;
    std::string TargetPosName;
    uint8_t TargetPosID;
    bool GripFlag;
    bool DropFlag;
    uint8_t SpeedLevel;

    std::string LLM_response;
};


struct Param 
{
    std::string APP_LLM_TOPIC;// "APP_LLM_Mesg"
    std::string ROBOT_CONTROL_TOPIC;// "APP_LLM_Mesg"
    std::string APP_LLM_Algo_TOPIC;
    int rosloop_rate;//100
    int rqt_test;
};


#endif