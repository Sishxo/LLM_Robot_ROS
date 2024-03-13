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
    /// @brief 结束
    void End();
public:
    /// @brief 获取ros node
    /// @return Node_
    ros::NodeHandle getNodeHandle();
    /// @brief 向大模型算法服务发送请求，获取模型处理结果
    /// @param msg 请求内容
    /// @param data 结果
    /// @return true表示成功，false表示失败
    bool LLM_Algo_Service(std::string msg, struct LLM_str &data);
    
private:
    ros::NodeHandle Node_;
    //APP发送的大模型请求服务
    ros::ServiceServer serviceLLM_ ;
    //发送机器人控制
    ros::Publisher  PubRobotControlMessage_;
    //llm算法服务
    ros::ServiceClient serviceLLM_Algo_;

private:
    /// @brief APP发送的大模型请求服务回调函数
    /// @param req 请求命令
    /// @param res 回馈命令
    /// @return 成功返回true，失败返回false
    bool APPServiceLLMCallback(commonType::APP_LLM::Request& req, commonType::APP_LLM::Response& res);

};




#endif