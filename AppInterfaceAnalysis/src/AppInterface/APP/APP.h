#ifndef APP_H_
#define APP_H_


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

public:
    /// @brief 预处理
    /// @param msg 
    void APP_PreProcess(std_msgs::String msg);


    // void get

private:

    std_msgs::String AllMessage;
};




#endif