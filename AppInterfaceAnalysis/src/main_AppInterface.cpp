
#include "common.h"

int main(int argc, char** argv)
{
    // 设置ROS_INFO输出中文
    setlocale(LC_CTYPE, "zh_CN.utf8");
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "AppInterfaceAnalysis_node");
    
    ros::Time::init();

    InterfaceAnalysis *appInterAna 
    =  new InterfaceAnalysis();

    appInterAna->Init();

    appInterAna->Start();

    appInterAna->End();

    delete(appInterAna);

}