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
    PubRobotControlMessage_ = Node_.advertise<commonType::RobotControl>(YamlConfig::Instance().GetParam().ROBOT_CONTROL_TOPIC, 1);
    //llm算法服务
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
    bool ret;
    //1.分析
    struct LLM_str LLM_Data;
    APP_LLM_REQUEST state = (APP_LLM_REQUEST)req.confirm_ID;
    std::string reqMsg = req.message;
    switch (state)
    {
        //app发送信息，app类开始叠加记录信息
        case APP_LLM_REQUEST_TranceMessage:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP::Instance().APP_ROSResponse_TranceMessage(req,res);
            break;
        }

        //app发送第一次请求，请求打模型对之前发送的信息进行处理分析
        //处理分析的结果发送到app
        case APP_LLM_REQUEST_FirstConfirm:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP::Instance().APP_ROSResponse_FirstConfirm(req,res,LLM_Data);
            break;
        }

        //app发送第二次请求，开始触发机器人控制
        case APP_LLM_REQUEST_SecondConfirm:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP::Instance().APP_ROSResponse_SecondConfirm(req,res,LLM_Data);

            //3.发送机器人控制
            commonType::RobotControl control;
            APP::Instance().getRobotControlMsg(LLM_Data, control);
            PubRobotControlMessage_.publish(control);
            break;
        }
        //异常
        default:
        {
            /* code */
            //异常处理
            ret = false;
            break;
        }
    }
    
    return ret;
}

/// @brief 向大模型算法服务发送请求，获取模型处理结果
/// @param msg 请求内容
/// @param data 结果
/// @return true表示成功，false表示失败
bool ROSControl::LLM_Algo_Service(std::string msg, struct LLM_str &data)
{
    commonType::APP_LLM_Algo srv;
    //发送服务请求
    if (serviceLLM_Algo_.call(srv))
    {
        return true;
    }else{
        return false;
    }
}