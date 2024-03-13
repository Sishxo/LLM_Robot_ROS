#include "YamlConfig.h"

void YamlConfig::Init() {
    // 参数加载
    out.APP_LLM_TOPIC = getParam<std::string>("/AppInterfaceAnalysis_node/Rostopic/APP_LLM_topic","APP_LLM_Mesg");// "APP_LLM_Mesg"
    out.ROBOT_CONTROL_TOPIC = getParam<std::string>("/AppInterfaceAnalysis_node/Rostopic/ROBOT_CONTROL_topic","RobotControl_Mesg");;// "APP_LLM_Mesg"
    out.rosloop_rate = getParam<int>("/AppInterfaceAnalysis_node/Rostopic/rosloop_rate",100);//100
    out.APP_LLM_Algo_TOPIC = getParam<std::string>("/AppInterfaceAnalysis_node/Rostopic/APP_LLM_Algo_topic","APP_LLM_Algo_topic");
}


/// @brief 结束
void YamlConfig::End()
{

}

struct Param YamlConfig::GetParam()
{
    return out;
}


//读取参数模板
template<typename T> T YamlConfig::getParam(const std::string& name,const T& defaultValue)
{
    T v;
    if(ros::param::get(name,v))//get parameter by name depend on ROS.
    {
        // ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
        return v;
    }else 
        // ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
}
