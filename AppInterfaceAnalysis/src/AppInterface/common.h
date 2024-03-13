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


/// @brief 打模型检测状态结果，成功，失败，超时...
enum LLM_State{

};

enum APP_LLM_REQUEST
{
    APP_LLM_REQUEST_TranceMessage = 0,
    APP_LLM_REQUEST_FirstConfirm = 1,
    APP_LLM_REQUEST_SecondConfirm = 2,
};

struct LLM_str {
    float speed;


    LLM_State state; 
};


struct Param 
{
    std::string APP_LLM_TOPIC;// "APP_LLM_Mesg"
    std::string ROBOT_CONTROL_TOPIC;// "APP_LLM_Mesg"
    std::string APP_LLM_Algo_TOPIC;
    int rosloop_rate;//100
};


#endif