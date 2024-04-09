#ifndef APP_H_
#define APP_H_

#include "common.h"
#include "commonType/interfaceAnalysisControl.h"

class APP
{
public:
    ~APP(){};
    APP(const APP&) = delete;
    APP& operator=(const APP&) = delete;
    static APP& Instance() {
        static APP m_pInstance;
        return m_pInstance;
    }
private:
    APP(){};
public:
    /// @brief 初始化
    void Init();
    /// @brief 开始
    void Start();
    /// @brief 结束
    void End();

public:
    /// @brief 对大模型发送的信息进行清空复位
    /// @param req 请求
    /// @param res 发送
    bool APP_ROSResponse_Reset(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res);
    /// @brief 对大模型发送的信息进行初级处理
    /// @param req 请求
    /// @param res 发送
    bool APP_ROSResponse_TranceMessage(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res);
    /// @brief 对大模型发送的信息进行第一次确定处理
    /// @param req 请求
    /// @param res 发送
    /// @param LLM_Data 处理结果
    bool APP_ROSResponse_FirstConfirm(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res);
    /// @brief 对大模型发送的信息进行第二次确定处理
    /// @param req 请求
    /// @param res 发送
    /// @param LLM_Data 处理结果
    bool APP_ROSResponse_SecondConfirm(commonType::APP_LLM::Request &req,commonType::APP_LLM::Response& res);
    /// @brief 将LLM_Data 转为commonType::RobotControl
    /// @param LLM_Data 
    /// @param control 
    void getRobotControlMsg(struct LLM_str &LLM_Data, commonType::RobotControl& control);
    /// @brief 信息调度Callback
    /// @param LLM_Data 
    /// @param control 
    bool APPServiceLLMCallback(commonType::APP_LLM::Request  &req, commonType::APP_LLM::Response &res);
private:
    /// @brief　储存经过LLM处理的指令vector
    std::vector<commonType::interfaceControl> first_LLM_Data_;
    /// @brief 存储每次有效app发送过来的叠加信息
    std::string AllAPPMessage_;
    /// @brief 预处理,将每次接收到的请求信息进行组包
    /// @param msg 
    void APP_preProcess(std::string msg);
    /// @brief 调用算法获语言识别的结果
    /// @return 返回语音识别的结果
    struct LLM_str APP_getAlgorithmResponse();
    /// @brief 将msg翻译成LLM_Data
    /// @param msg 收入字符串
    /// @param LLM_Data 输出控制命令
    /// @return true表示成功，false表示失败
    bool messageToROSResponse(std::string msg, struct LLM_str &LLM_Data);

};




#endif