#include "Xarm_Rotate_Controller.h"

#define M_M_PI 3.1

namespace XARMGRIP_CONTROLLER
{

    /// @brief 初始化
    void XarmRotate::Init()
    {
        
    }
    /// @brief 关闭
    void XarmRotate::Close()
    {

    }

    /// @brief 周期巡检
    /// @param startPos 起始位置 
    /// @param objectID 目标检测物体id
    /// @return true，表示目标检测成功，false，表示目标检测失败
    bool XarmRotate::CycleRotate(Xarm6Position startPos, int objectID)
    {
        float deltAngle = ParamROS::Instance().GetParam().rotateDeltAngle * M_PI / 180;
        commonType::TargetReco target_msg;
        //正向
        for(;;)
        {
            Xarm6Position current_Pos = XarmGripROS::Instance().GetXarmLocation();
            Xarm6Position next_pos = current_Pos;
            if((next_pos.joint[0] + deltAngle) >= M_M_PI)
            {
                next_pos.joint[0] = M_M_PI;
                if(Rotate(next_pos)== false) 
                    return false;
                else{
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID,  target_msg) == true) 
                        return true;
                    deltAngle = deltAngle * (-1);
                    break;
                } 

            }else{
                if(AngleRotate(current_Pos, deltAngle) == false) 
                    return false;
                else{
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID,  target_msg) == true) 
                        return true;
                }
            }
        }
        //反向
        for(;;)
        {
            Xarm6Position current_Pos = XarmGripROS::Instance().GetXarmLocation();
            Xarm6Position next_pos = current_Pos;
            if((next_pos.joint[0] + deltAngle) <= (M_M_PI * -1))
            {
                next_pos.joint[0] = (M_M_PI * (-1));
                if(Rotate(next_pos)== false) 
                    return false;
                else{
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID,  target_msg) == true) 
                        return true;
                    else {
                        //归位
                        Rotate(startPos);
                        return false;
                    }         
                } 
            }else{
                if(AngleRotate(current_Pos, deltAngle) == false) 
                    return false;
                else{
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID,  target_msg) == true) 
                        return true;
                }
            }
        }
        return true;
    }
    /// @brief 角度旋转
    /// @param currentPos 当前位置
    /// @param angle 旋转角度
    /// @return true表示旋转成功，false表示旋转失败
    bool XarmRotate::AngleRotate(Xarm6Position &currentPos, float angle)
    {
        float currentAngle = currentPos.joint[0];
        float nextAngle = currentAngle + angle;
        Xarm6Position nextPos = currentPos;
        nextPos.joint[0] = nextAngle;
        ros::Time startTime = ros::Time::now() ;
        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);

        return Rotate(nextPos);
    }

    /// @brief 判断当前位置与目标位置的角度是否在阈值以内
    /// @param pos1 
    /// @param pos2 
    /// @param mistake 
    /// @return 
    bool XarmRotate::CheckAngle(Xarm6Position pos1, Xarm6Position pos2, float mistake)
    {
        int size = 6;
        int cout = 0 ;
        float mistake_new = mistake * M_PI / 180;


        float dis[6];

        for(int i = 0 ; i < size ; i++)
        {
            dis[i] = fabs(pos1.joint[i] - pos2.joint[i]);
        }
        if((dis[0] < mistake_new) && (dis[1] < mistake_new) && (dis[2] < mistake_new)
            && (dis[3] < mistake_new) && (dis[4] < mistake_new) && (dis[5] < mistake_new))
        {
            return true;
        }else return false;
    }

    /// @brief 旋转到目标位置
    /// @param targetPos 
    /// @return 
    bool XarmRotate::Rotate(Xarm6Position targetPos)
    {
        XarmGripROS::Instance().XarmControlModelSet(ParamROS::Instance().GetParam().XarmRotateModel);
        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);
        ros::Time startTime = ros::Time::now() ;
        targetPos.jointControl = true;
        XarmGripROS::Instance().XarmControl(targetPos);

        // 等待图像检测结果
        while(ros::ok())
        {
            if(CheckAngle(targetPos, XarmGripROS::Instance().GetXarmLocation(), ParamROS::Instance().GetParam().rotateMistake) == true) {
                break;
            }else{

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

    /// @brief 旋转到目标位置所在角度
    /// @param targetPos 
    /// @return 
    bool XarmRotate::RotatetoTargetAngle(Xarm6Position targetPos)
    {
        Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();
        float angle = GetDeltAngle(startPos, targetPos);
        return AngleRotate(startPos, angle);
    }
    
    /// @brief 计算pos1到pos2所在位置需要旋转多少角度
    /// @param Pos1 
    /// @param Pos2 
    /// @return 
    float XarmRotate::GetDeltAngle(Xarm6Position Pos1, Xarm6Position Pos2)
    {
        float angle2 = atan2(Pos2.y, Pos2.x);
        float angle1 = atan2(Pos1.y, Pos1.x);
        return angle2 - angle1;
    }

} 
