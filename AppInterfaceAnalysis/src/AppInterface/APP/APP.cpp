#include "APP.h"
#include "YamlConfig.h"

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
    AllAPPMessage_ += msg;
}

/// @brief 对大模型发送的信息进行清空复位
/// @param req 请求
/// @param res 发送
bool APP::APP_ROSResponse_Reset(commonType::APP_LLM::Request &req, commonType::APP_LLM::Response &res)
{
    Init();
    res.ret=0;
    res.message=AllAPPMessage_;
    res.command = false;
    return true;
}

/// @brief 对大模型发送的信息进行初级处理
/// @param req 请求
/// @param res 发送
bool APP::APP_ROSResponse_TranceMessage(commonType::APP_LLM::Request &req, commonType::APP_LLM::Response &res)
{

    APP_preProcess(req.message);

    res.message = AllAPPMessage_;
    res.ret = 1;
    res.command = false;

    return true;
}

/// @brief 对大模型发送的信息进行第一次确定处理
/// @param req 请求
/// @param res 发送
/// @param LLM_Data 处理结果
bool APP::APP_ROSResponse_FirstConfirm(commonType::APP_LLM::Request &req, commonType::APP_LLM::Response &res)
{
    std::vector<commonType::interfaceControl> LLM_Data;
    std::string appMessage = AllAPPMessage_; // 获取app发送的所有信息
    std::string LLM_response;
    uint8_t action_type;
    bool ret = ROSControl::Instance().LLM_Algo_Service(appMessage, LLM_Data, LLM_response, action_type);

    res.ret = 2;
    if (action_type == 5)
    {
        res.command = false;
    }
    else
    {
        res.command = true;
    }

    res.control.resize(LLM_Data.size());
    for (int i = 0; i < LLM_Data.size(); i++)
    {
        res.control[i] = LLM_Data[i];
    }

    res.message = LLM_response;

    res.ActionType = action_type;

    first_LLM_Data_ =  LLM_Data;

    // if (ret)
    // {
    //     AllAPPMessage_ = "";
    // }
    return true;
}

/// @brief 对大模型发送的信息进行第二次确定处理
/// @param req 请求
/// @param res 发送
/// @param LLM_Data 处理结果
bool APP::APP_ROSResponse_SecondConfirm(commonType::APP_LLM::Request &req, commonType::APP_LLM::Response &res)
{
    std::string appMessage = req.message;
    int confirm_ID = req.confirm_ID;

    if (confirm_ID == 0)
    {
        AllAPPMessage_ = "";
        return false;
    }

    AllAPPMessage_ = "";

    res.ret = 3;
    commonType::interfaceAnalysisControl control;
    control.request.control.resize(req.control.size());
    for(int i=0; i<req.control.size(); i++){
        control.request.control[i]=req.control[i];
    }

    if(YamlConfig::Instance().GetParam().rqt_test == 1)
    {
        control.request.control.resize(first_LLM_Data_.size());
        for(int i = 0 ; i < first_LLM_Data_.size(); i++)
        {
            control.request.control[i] = first_LLM_Data_[i];
        }
    }

    ROSControl::Instance().sendServiceDispach(control);

    return true;
}

bool APP::APPServiceLLMCallback(commonType::APP_LLM::Request  &req, commonType::APP_LLM::Response &res){

    bool ret;
    //1.分析
    struct LLM_str LLM_Data;
    APP_LLM_REQUEST state = (APP_LLM_REQUEST)req.confirm_ID;
    std::string reqMsg = req.message;
    switch (state)
    {
        //app发送信息，app类开始叠加记录信息
        case APP_LLM_REQUEST_Reset:
        {
            ret = APP_ROSResponse_Reset(req,res);
            break;
        }

        case APP_LLM_REQUEST_TranceMessage:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP_ROSResponse_TranceMessage(req,res);
            break;
        }

        //app发送第一次请求，请求大模型对之前发送的信息进行处理分析
        //处理分析的结果发送到app
        case APP_LLM_REQUEST_FirstConfirm:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP_ROSResponse_FirstConfirm(req,res);
            break;
        }

        //app发送第二次请求，开始触发机器人控制
        case APP_LLM_REQUEST_SecondConfirm:
        {
            /* code */
            //2.调用app类完成处理
            ret = APP_ROSResponse_SecondConfirm(req,res);
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