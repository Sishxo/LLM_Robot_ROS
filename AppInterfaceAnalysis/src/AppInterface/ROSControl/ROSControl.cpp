#include "ROSControl.h"
#include "YamlConfig.h"


// #define APP_LLM_TOPIC "APP_LLM_Mesg"
// #define ROBOT_CONTROL_TOPIC "APP_LLM_Mesg"

/// @brief 初始化
void ROSControl::Init()
{
    ROS_INFO("ROSControl初始化...............");
    Node_ = YamlConfig::Instance().GetRosNode();
    //APP发送的大模型请求服务
    serviceLLM_ = Node_.advertiseService(YamlConfig::Instance().GetParam().APP_LLM_TOPIC, &ROSControl::APPServiceLLMCallback, this);
    //发送机器人控制
    serviceDispach_ = Node_.serviceClient<commonType::interfaceAnalysisControl>(YamlConfig::Instance().GetParam().ROBOT_CONTROL_TOPIC);
    //llm算法服务
    ros::service::waitForService(YamlConfig::Instance().GetParam().APP_LLM_Algo_TOPIC);
    serviceLLM_Algo_ = Node_.serviceClient<commonType::APP_LLM_Algo>(YamlConfig::Instance().GetParam().APP_LLM_Algo_TOPIC);
}

/// @brief 开始
void ROSControl::Start()
{
    ros::Rate loop_rate(YamlConfig::Instance().GetParam().rosloop_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

/// @brief 结束
void ROSControl::End()
{

}

/// @brief 获取ros node
/// @return Node_
ros::NodeHandle ROSControl::getNodeHandle()
{
    return Node_;
}

/// @brief APP发送的大模型请求服务回调函数
/// @param req 请求命令
/// @param res 回馈命令
/// @return 成功返回true，失败返回false
bool ROSControl::APPServiceLLMCallback(commonType::APP_LLM::Request  &req, commonType::APP_LLM::Response &res)
{
    return APP::Instance().APPServiceLLMCallback(req,res);   
}

/// @brief 向大模型算法服务发送请求，获取模型处理结果
/// @param msg 请求内容
/// @param data 结果
/// @return true表示成功，false表示失败
bool ROSControl::LLM_Algo_Service(std::string msg, std::vector<commonType::interfaceControl> &data, std::string &response, uint8_t &action_type)
{
    commonType::APP_LLM_Algo srv;
    srv.request.message = msg;
    //发送服务请求
    if (serviceLLM_Algo_.call(srv))
    {
        data.clear();
        for(int i = 0 ; i< srv.response.control.size(); i++)
        {
            data.push_back(srv.response.control[i]);
        }
        response = srv.response.message;
        action_type = srv.response.ActionType;
        return true;
    }else{
        return false;
    }
}

bool ROSControl::sendServiceDispach(commonType::interfaceAnalysisControl &data)
{
    //发送服务请求
    if (serviceDispach_.call(data))
    {
        return true;
    }else{
        return false;
    }
}