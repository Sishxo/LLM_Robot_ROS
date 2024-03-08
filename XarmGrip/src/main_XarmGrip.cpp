#include "ros/ros.h"
#include "XarmGrip_Controller.h"


int main(int argc, char** argv)
{
    // 设置ROS_INFO输出中文
    setlocale(LC_CTYPE, "zh_CN.utf8");
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "xarm6Grip_node");
    ros::Time::init();
    // 定义
    XARMGRIP_CONTROLLER::XarmGrip *Controller 
        = new XARMGRIP_CONTROLLER::XarmGrip();
    //初始化
    Controller->Init();
    // 启动
    Controller->Start();
    // 释放资源
    delete(Controller);
}