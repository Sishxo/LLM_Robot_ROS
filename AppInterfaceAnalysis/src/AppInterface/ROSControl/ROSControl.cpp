#include "ROSControl.h"


#define APP_TOPIC "APP_LLM_Mesg"

/// @brief 初始化
void ROSControl::Init()
{
    ROS_INFO("ROSControl初始化...............");

    SubAPPLLM_ = Node_.subscribe(APP_TOPIC, 1, &ROSControl::APPLLMCallback, this);
}

/// @brief 开始
void ROSControl::Start()
{
}

/// @brief 获取ros node
/// @return Node_
ros::NodeHandle ROSControl::getNodeHandle()
{
    return Node_;
}


/// @brief APP标定回调检测函数
void ROSControl::APPLLMCallback(const std_msgs::StringConstPtr& LLm_msg)
{

}