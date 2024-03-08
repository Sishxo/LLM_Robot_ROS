#ifndef ROSCONTROL_H_
#define ROSCONTROL_H_

#include "common.h"

/// @brief 
class ROSControl
{
public:
    ~ROSControl(){};
    ROSControl(const ROSControl&) = delete;
    ROSControl& operator=(const ROSControl&) = delete;
    static ROSControl& Instance() {
        static ROSControl m_pInstance;
        return m_pInstance;
    }
private:
    ROSControl(){};
public:
    /// @brief 初始化
    void Init();
    /// @brief 开始
    void Start();

public:
    /// @brief 获取ros node
    /// @return Node_
    ros::NodeHandle getNodeHandle();
    
private:
    ros::NodeHandle Node_;
    ros::Subscriber SubAPPLLM_;

private:
    /// @brief APP获取到的原始数据流
    void APPLLMCallback(const std_msgs::StringConstPtr& LLm_msg);

};




#endif