#ifndef INTERFACEANALYSIS_H_
#define INTERFACEANALYSIS_H_

#include "common.h"

class InterfaceAnalysis
{
public:
    ~InterfaceAnalysis(){};
    InterfaceAnalysis(const InterfaceAnalysis&) = delete;
    InterfaceAnalysis& operator=(const InterfaceAnalysis&) = delete;
    static InterfaceAnalysis& Instance() {
        static InterfaceAnalysis m_pInstance;
        return m_pInstance;
    }
private:
    InterfaceAnalysis(){};
public:
    /// @brief 初始化
    void Init();
    /// @brief 开始
    void Start();
    /// @brief 执行
    void DoFunction();
    /// @brief 结束
    void End();
};




#endif