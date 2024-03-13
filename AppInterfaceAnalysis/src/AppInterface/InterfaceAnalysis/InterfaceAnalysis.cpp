#include "InterfaceAnalysis.h"
#include "YamlConfig.h"

//构造函数
InterfaceAnalysis::InterfaceAnalysis()
{
    YamlConfig::Instance().Init();
    ROSControl::Instance().Init();
    APP::Instance().Init();
}
//析构函数
InterfaceAnalysis::~InterfaceAnalysis()
{
    YamlConfig::Instance().End();
    ROSControl::Instance().End();
    APP::Instance().End();
}

/// @brief 初始化
void InterfaceAnalysis::Init()
{
    ROSControl::Instance().Init();
}
/// @brief 开始
void InterfaceAnalysis::Start()
{
    ROSControl::Instance().Start();
}   
/// @brief 结束
void InterfaceAnalysis::End()
{

}
