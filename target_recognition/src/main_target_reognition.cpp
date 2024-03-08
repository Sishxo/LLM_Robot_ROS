#include "ros/ros.h"
#include "target_recognition.h"
#include <pybind11/embed.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_recognition_node");

    pybind11::scoped_interpreter guard{};
    // 定义
    Target_Recognition::ROS_Target_Recognition *targetRecog 
        = new Target_Recognition::ROS_Target_Recognition();
    // 启动
    targetRecog->Start();
    // 释放资源
    delete(targetRecog);
}