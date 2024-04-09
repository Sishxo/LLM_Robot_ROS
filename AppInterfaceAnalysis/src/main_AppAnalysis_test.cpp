#include "ros/ros.h"
#include "commonType/APP_LLM_Algo.h"
 
// 服务回调函数，计算请求的两个数的和
bool myServiceCallback(commonType::APP_LLM_Algo::Request  &req,
                      commonType::APP_LLM_Algo::Response &res) {
    ROS_INFO("大模型收到信息...............");
    res.control.resize(4);

    // uint8 ObjectID  #控制物体，0表示车，1表示机械臂
    // string start    #控制物体移动的开始点，空表示从当前位置出发
    // string end      #控制物体移动的结束点，空表示不移动
    // float32 speed   #控制物体移动的速度，0表示不移动
    // uint8 gripFunction    #控制物体操作，特别指定了机械臂膀，0表示抓，1表示放
    // uint8 gripObjectID  #控制机械臂抓取的物体id
    // float32 delyaTime   #动作完成后等待时间
    res.control[0].ObjectID = 0;
    res.control[0].start = "a";
    res.control[0].end = "b";
    res.control[0].speed = 0.1;
    res.control[0].delyaTime = 1.0;

    res.control[1].ObjectID = 1;
    res.control[1].start = "home";
    res.control[1].end = "c";
    res.control[1].speed = 0.1;
    res.control[1].gripFunction = 1;
    res.control[1].gripObjectID = 1;
    res.control[1].delyaTime = 1.0;

    res.control[2].ObjectID = 0;
    res.control[2].start = "b";
    res.control[2].end = "a";
    res.control[2].speed = 0.1;
    res.control[2].delyaTime = 1.0;

    res.control[3].ObjectID = 1;
    res.control[3].start = "home";
    res.control[3].end = "d";
    res.control[3].speed = 0.1;
    res.control[3].gripFunction = 0;
    res.control[3].gripObjectID = 1;
    res.control[3].delyaTime = 1.0;
    return true;
}
 
int main(int argc, char **argv) {


    // 设置ROS_INFO输出中文
    setlocale(LC_CTYPE, "zh_CN.utf8");
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "my_service_server");
 
    // 创建节点句柄
    ros::NodeHandle n;
 
    // 注册服务server，并设置回调函数
    ros::ServiceServer service = n.advertiseService("/APP_LLM_Algo_topic", myServiceCallback);
    ROS_INFO("Ready to receive requests.");
 
    // 循环等待回调函数
    ros::spin();
 
    return 0;
}