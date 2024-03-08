#include "GripObject.h"

namespace XARMGRIP_CONTROLLER
{
    /// @brief 初始化
    void GripObject::Init()
    {

    }
    /// @brief 关闭
    void GripObject::Close()
    {

    }

    /// @brief 开始抓取
    /// @param objectPos 
    bool GripObject::StartGrip()
    {
        if(XarmDiKer::Instance().DikerControl(prefabPos_) == false) return false;
        return true;
    }
    /// @brief 前往目标点
    /// @return 
    bool GripObject::GoGrip()
    {
        commonType::TargetReco target_msg;
        XarmGripROS::Instance().GetTargetRecognition(0, target_msg) ;
        XarmGripROS::Instance().XarmCameraTF(target_msg);                      
        Xarm6Position targetPos = Realtest(target_msg);
        if(XarmDiKer::Instance().DikerControl(targetPos) == false) return false;
        return true;
    }

    /// @brief 前往目标点
    /// @return 
    bool GripObject::GoRelease()
    {
        if(XarmDiKer::Instance().DikerControl(target_) == false) return false;
        return true;
    }
    /// @brief 从目标点回
    /// @return 
    bool GripObject::BackGrip()
    {
        if(XarmDiKer::Instance().DikerControl(prefabPos_) == false) return false;
        return true;
    }

    /// @brief 
    /// @return 
    bool GripObject::EndGrip()
    {
        if(XarmDiKer::Instance().DikerControl(startPos_) == false) return false;
        return true;
    }



    /// @brief 前往目标点
    /// @param objectPos 
    /// @return 
    bool GripObject::Go(Xarm6Position startPos, Xarm6Position target)
    {
        startPos_ = startPos;
        target_ = target;
        prefabPos_ = GetPrefab(startPos, target);

        if(StartGrip() == false) return false;

        if(GoGrip() == false) return false;
        return true;
    }

    bool GripObject::Release(Xarm6Position startPos, Xarm6Position target)
    {
        startPos_ = startPos;
        target_ = target;
        prefabPos_ = GetPrefab(startPos, target);

        if(StartGrip() == false) return false;

        if(GoRelease() == false) return false;
        return true;
    }

    /// @brief 抓取
    /// @return 
    bool GripObject::Grip()
    {
        GripControl::Instance().GripClose(620);
        return true;
    }

    /// @brief 放置
    /// @return 
    bool GripObject::UnGrip()
    {
        int size = (int)ParamROS::Instance().GetParam().grip_open;
        GripControl::Instance().GripClose(size);
        return true;
    }

    /// @brief 回到初始位置
    /// @return 
    bool GripObject::Back()
    {
        if(BackGrip() == false) return false;
        if(EndGrip() == false) return false;
        return true;
    }

    Xarm6Position GripObject::GetPrefab(Xarm6Position xarm6, Xarm6Position target)
    {
        Xarm6Position out = xarm6;
        float distance = GetDistance(xarm6, target);
        // out.x = (xarm6.x - target.x) / distance * STRAIGNT_LEN + target.x;
        // out.y = (xarm6.y - target.y) / distance * STRAIGNT_LEN + target.y;
        out.x = (xarm6.x - target.x) / 2 + target.x;
        out.y = (xarm6.y - target.y) / 2 + target.y;
        out.z = target.z;
        out.roll = target.roll;
        out.pitch = target.pitch;
        out.yaw = target.yaw;
        return out;
    }

    float GripObject::GetDistance(Xarm6Position Target, Xarm6Position Xarm)
    {
        float dis_x = (Target.x - Xarm.x) * (Target.x - Xarm.x);
        float dis_y = (Target.y - Xarm.y) * (Target.y - Xarm.y);
        float dis_z = (Target.z - Xarm.z) * (Target.z - Xarm.z);
        float dis = sqrt(dis_x + dis_y + dis_z);      
        return dis;
    }

        /// @brief 回到坐标原点
    Xarm6Position GripObject::GoHome()
    {
        Param Paramdata = ParamROS::Instance().GetParam();
        Xarm6Position targetPos;
        targetPos.joint[0] = 0;
        targetPos.joint[1] = -1.188538;
        targetPos.joint[2] = -0.312190;
        targetPos.joint[3] = -3.1415;
        targetPos.joint[4] = 0.06902146339416504;
        targetPos.joint[5] = 3.1415;
        return targetPos;
//         XarmRotate::Instance().Rotate(targetPos);
// #if GRIP_USE
//         GripControl::Instance().GripOpen();
// #endif
    }
    
    Xarm6Position GripObject::Realtest(commonType::TargetReco target_msg)
    {
        float yaml = XarmGripROS::Instance().GetXarmLocation().joint[0];
        Xarm6Position XarmGrip;
        XarmGrip.x = target_msg.x * 1000;
        XarmGrip.y = target_msg.y * 1000;
        XarmGrip.z = target_msg.z * 1000;
        XarmGrip.roll = 3.1415;
        XarmGrip.pitch = -1.5707;
        XarmGrip.yaw = 0;
        
        //选找到目标角度
        float angle = XarmRotate::Instance().RotatetoTargetAngle(XarmGrip);
        XarmGrip.yaw = XarmGripROS::Instance().GetXarmLocation().joint[0];
        return XarmGrip;
    }
}