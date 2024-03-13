#ifndef YAMLCONFIG_H_
#define YAMLCONFIG_H_

#include "common.h"

class YamlConfig
{
public:
    ~YamlConfig(){};
    YamlConfig(const YamlConfig&) = delete;
    YamlConfig& operator=(const YamlConfig&) = delete;
    static YamlConfig& Instance() {
        static YamlConfig m_pInstance;
        return m_pInstance;
    }
private:
    YamlConfig(){};

public:
    void End();
    struct Param GetParam();
    /// @brief 初始化
    void Init();
    ros::NodeHandle GetRosNode()
    {
        return Node_;
    }
private:
    template<typename T> T getParam(const std::string& name,const T& defaultValue);
    ros::NodeHandle Node_;
    struct Param out;

};

#endif