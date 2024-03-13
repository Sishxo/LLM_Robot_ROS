#ifndef ALGORITHM_H_
#define ALGORITHM_H_

/// @brief 大模型算法
class Algorithm
{

public:
    ~Algorithm(){};
    Algorithm(const Algorithm&) = delete;
    Algorithm& operator=(const Algorithm&) = delete;
    static Algorithm& Instance() {
        static Algorithm m_pInstance;
        return m_pInstance;
    }
private:
    Algorithm(){};


public:
    /// @brief 初始化
    void Init();
    /// @brief 开始
    void Start();

};


#endif