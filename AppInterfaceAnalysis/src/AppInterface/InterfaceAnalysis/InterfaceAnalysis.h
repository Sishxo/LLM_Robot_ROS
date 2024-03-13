#ifndef INTERFACEANALYSIS_H_
#define INTERFACEANALYSIS_H_

#include "common.h"

class InterfaceAnalysis
{
public:
    //构造函数
    InterfaceAnalysis();
    //析构函数
    ~InterfaceAnalysis();
public:
    /// @brief 初始化
    void Init();
    /// @brief 开始
    void Start();
    /// @brief 结束
    void End();
};




#endif