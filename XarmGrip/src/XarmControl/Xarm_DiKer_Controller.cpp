#include "Xarm_DiKer_Controller.h"

namespace XARMGRIP_CONTROLLER
{

    /// @brief 初始化
    void XarmDiKer::Init()
    {
        
    }
    /// @brief 关闭
    void XarmDiKer::Close()
    {

    }

    float XarmDiKer::GetDistance(Xarm6Position pos1, Xarm6Position pos2)
    {
        float xx  = pos1.x - pos2.x;
        float yy  = pos1.y - pos2.y;
        float zz  = pos1.z - pos2.z;
        return sqrt(xx * xx + yy * yy + zz * zz);
    }

    /// @brief 根据目标点，采用笛卡尔坐标系控制
    /// @param pos 目标坐标
    /// @return true，表示到达目的地，false，表示异常
    bool XarmDiKer::DikerControl(Xarm6Position pos)
    {
        XarmGripROS::Instance().XarmControlModelSet(ParamROS::Instance().GetParam().XarmControlModel);
        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);
        ros::Time startTime = ros::Time::now() ;
        pos.jointControl == false;
        XarmGripROS::Instance().XarmControl(pos);
        // 等待图像检测结果
        while(ros::ok())
        {      
            if(GetDistance(pos, XarmGripROS::Instance().GetXarmLocation()) <= ParamROS::Instance().GetParam().controlMistake)
            {
                break;
            }
            // ros::Duration deltTime = ros::Time::now() - startTime;
            // if(deltTime.toSec() >= ParamROS::Instance().GetParam().OverTime)
            // {
            //     return false;
            // }
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }

} 