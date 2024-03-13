#include "APP.h"

void APP::Init()
{
    AllAPPMessage_ = "";
}

void APP::Start()
{

}

/// @brief 结束
void APP::End()
{

}

/// @brief 预处理
/// @param msg 
void APP::APP_preProcess(std::string msg)
{
    AllAPPMessage_  += msg;
}

/// @brief 对大模型发送的信息进行初级处理
/// @param req 请求
/// @param res 发送
bool APP::APP_ROSResponse_TranceMessage(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res)
{
    return true;
}
/// @brief 对大模型发送的信息进行第一次确定处理
/// @param req 请求
/// @param res 发送
/// @param LLM_Data 处理结果
bool APP::APP_ROSResponse_FirstConfirm(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res, struct LLM_str &LLM_Data)
{

    std::string appMessage = AllAPPMessage_;    //获取app发送的所有信息
    return ROSControl::Instance().LLM_Algo_Service(appMessage, LLM_Data);
}
/// @brief 对大模型发送的信息进行第二次确定处理
/// @param req 请求
/// @param res 发送
/// @param LLM_Data 处理结果
bool APP::APP_ROSResponse_SecondConfirm(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res, struct LLM_str &LLM_Data)
{
    std::string appMessage = req.message;    //获取app发送的所有信息
    AllAPPMessage_ = "";
    return messageToROSResponse(appMessage, LLM_Data);
}

/// @brief 将LLM_Data 转为commonType::RobotControl
/// @param LLM_Data 
/// @param control 
void APP::getRobotControlMsg(struct LLM_str &LLM_Data, commonType::RobotControl& control)
{

}
/// @brief 将msg翻译成LLM_Data
/// @param msg 收入字符串
/// @param LLM_Data 输出控制命令
/// @return true表示成功，false表示失败
bool APP::messageToROSResponse(std::string msg, struct LLM_str &LLM_Data)
{
    return true;
}