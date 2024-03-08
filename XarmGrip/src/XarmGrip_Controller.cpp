#include "XarmGrip_Controller.h"


#define TEST  0

namespace XARMGRIP_CONTROLLER
{

    XarmGrip::XarmGrip()
    {
        ROS_INFO("机械臂控制开始...............");
        ParamROS::Instance().Init();
        XarmGripROS::Instance().Init();
        GripControl::Instance().Init();
        XarmDiKer::Instance().Init();
        XarmRotate::Instance().Init();
        GripObject::Instance().Init();
        ConfirmPoint::Instance().Init();

        // ConfirmPoint::Instance().test();
    }
    XarmGrip::~XarmGrip()
    {
        GripControl::Instance().Close();
        XarmDiKer::Instance().Close();
        XarmRotate::Instance().Close();
        GripObject::Instance().Close();
        ConfirmPoint::Instance().Close();
    }

    /// @brief 初始化
    void XarmGrip::Init()
    {
        Xarm6Position home = GripObject::Instance().GoHome();
        XarmRotate::Instance().Rotate(home);
        ROS_INFO("机械臂回到原位...............");
    }

        /// @brief 开始
    void XarmGrip::Start()
    {
        APPControl APPControlCommand;
        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();

            APPXarmGripControlState APPCommand = APPClient::Instance().GetAPPControlCommand(APPControlCommand);

            switch (APPCommand)
            {
                case APPXarmGripControl_NULL:
                {
                    APPClient::Instance().APPDONull(APPControlCommand);
                    break;
                }

                case APPXarmGripControl_GRIP:
                {
                    ROS_INFO("APP发送机械臂膀抓取控制指令...............");
                    APPClient::Instance().APPDoGrip(APPControlCommand);
                    break;
                }

                case APPXarmGripControl_CONFIRM:
                {
                    ROS_INFO("APP发送机械臂膀定位控制指令...............");
                    APPClient::Instance().APPDoConfirm(APPControlCommand);
                    break;
                }    
                default:
                    break;
            }

        }

    }
    // /// @brief 开始
    // void XarmGrip::Start()
    // {
    //     int objectID = 0;
    //     ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);
    //     XarmGripState state = XarmGripState_Null;
    //     // XarmGripState state = XarmGripState_start;
    //     commonType::TargetReco target_msg;
    //     commonType::XarmControlCommond control;

    //     commonType::TargetReco tempUnGripPos;

    //     APPControl APPControlCommand;

    //     while(ros::ok())
    //     {
    //         ros::spinOnce();
    //         loop_rate.sleep();

    //         APPXarmGripControlState APPCommand = XarmGripROS::Instance().GetAPPControlCommand(APPControlCommand);

    //         // switch (APPCommand)
    //         // {
    //         //     case APPXarmGripControl_NULL:
    //         //     {
    //         //         /* code */
    //         //         APPDONull(APPControlCommand);
    //         //         break;
    //         //     }

    //         //     case APPXarmGripControl_GRIP:
    //         //     {
    //         //         /* code */
    //         //         APPDoGrip(APPControlCommand);
    //         //         break;
    //         //     }

    //         //     case APPXarmGripControl_CONFIRM:
    //         //     {
    //         //         /* code */
    //         //         APPDoConfirm(APPControlCommand);
    //         //         break;
    //         //     }

    //         //     default:
    //         //         break;
    //         // }

    //         switch (state)
    //         {
    //             //检测是否有控制指令下发，如果有进入start
    //             case XarmGripState_Null:
    //             {
    //                 if(XarmGripROS::Instance().GetClientControlCommand(control) == false)
    //                 {
    //                     state = XarmGripState_Null;
    //                 }else{
    //                     state = XarmGripState_start;
    //                 }
    //                 break;
    //             }
    //             //在当前位置获取目标检查结果，如果发现目标则进入控制模式，如果没有则进入巡检
    //             case XarmGripState_start:
    //             {
    //                 ROS_INFO("开始...............");
    //                 if(CheckXarmCurrentState(control) == XarmGripState_Home)
    //                 {
    //                     state = XarmGripState_Home;
    //                 }else{
    //                     if(XarmGripROS::Instance().GetTargetRecognition(objectID, target_msg) == true)
    //                     {
    //                         state = CheckXarmCurrentState(control);
    //                     }
    //                     else {
    //                         state = XarmGripState_Cycle;
    //                     }
    //                 }
    //                 break;
    //             }
    //             //回到home位置
    //             case XarmGripState_Home:
    //             {
    //                 Init();
    //                 state = XarmGripState_Null;
    //                 break;
    //             }
    //             //巡检
    //             case XarmGripState_Cycle:  
    //             {
    //                 ROS_INFO("巡检...............");
    //                 Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();
    //                 if(XarmRotate::Instance().CycleRotate(startPos, objectID) == true)
    //                 {
    //                     ROS_INFO("巡检检测到目标...............");
    //                     state = CheckXarmCurrentState(control);
    //                 }else{
    //                     ROS_INFO("巡检未检测到目标,回到home...............");
    //                     state = XarmGripState_Null;
    //                     Init();
    //                 }
    //                 break;    
    //             }   
    //             //抓取
    //             case XarmGripState_Grip:
    //             {
    //                 ROS_INFO("抓取...............");
    //                 commonType::TargetReco Pos;
    //                 if(DoGrip(objectID, Pos) == false) 
    //                 {
    //                     ROS_INFO("抓取异常，回到原位...............");
    //                     Init();
    //                     state = XarmGripState_Null;
    //                 }else{
    //                     ROS_INFO("抓取成功...............");
    //                     state = XarmGripState_Null;
    //                 }       
    //                 break;
    //             }
    //             //抓取+释放
    //             case XarmGripState_UnGrip:
    //             {
    //                 ROS_INFO("先抓取，后放置...............");
    //                 commonType::TargetReco Pos;
    //                 // commonType::TargetReco target_msg;
    //                 if(DoGrip(objectID, Pos) == false) 
    //                 {
    //                     ROS_INFO("抓取异常，回到原位...............");
    //                     Init();
    //                     state = XarmGripState_Null;
    //                     break;
    //                 }else{
    //                     ROS_INFO("抓取成功...............");
    //                 }

    //                 if(DoRelease(Pos) == false)
    //                 {
    //                     ROS_INFO("放置异常，回到原位...............");
    //                     Init();
    //                     state = XarmGripState_Null;
    //                 }else{
    //                     ROS_INFO("放置成功...............");
    //                     state = XarmGripState_Null;
    //                 }
    //                 break;  
    //             }
    //         }
    //     }
    // }


    // int XarmGrip::APPDONull(APPControl &APPCommand)
    // {
    //     return GripClient::Instance().DoClinetNull(APPCommand);
    // }

    // int XarmGrip::APPDoGrip(APPControl &APPCommand)
    // {
    //     return GripClient::Instance().DoClientGrip(APPCommand);
    // }

    // int XarmGrip::APPDoConfirm(APPControl &APPCommand)
    // {
    //     return GripClient::Instance().DoClientConfirm(APPCommand);
    // }

    bool XarmGrip::DoGrip(int objectID, commonType::TargetReco& target_msg)
    {
        // commonType::TargetReco target_msg;
        //获取起始坐标
        Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();

        XarmGripROS::Instance().GetTargetRecognition(objectID, target_msg) ;
        XarmGripROS::Instance().XarmCameraTF(target_msg);
                        
        Xarm6Position targetPos = Realtest(startPos, target_msg);
        //获取当前坐标
        Xarm6Position startPos1 = XarmGripROS::Instance().GetXarmLocation();
        //运动到object位置
        if(GripObject::Instance().Go(startPos1, targetPos) == false) return false;
        //抓取
        if(GripObject::Instance().Grip() == false) return false;
        //收回
        if(GripObject::Instance().Back() == false) return false;
        //回到最开始位置
        XarmRotate::Instance().Rotate(startPos);

        return true;
    }

    bool XarmGrip::DoRelease(commonType::TargetReco Pos)
    {
        //获取起始坐标
        Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();                    
        Xarm6Position targetPos = Realtest(startPos, Pos);
        //运动到object位置
        if(GripObject::Instance().Go(startPos, targetPos) == false) return false;
        //抓取
        if(GripObject::Instance().UnGrip() == false) return false;
        //收回
        if(GripObject::Instance().Back() == false) return false;
        //回到最开始位置
        XarmRotate::Instance().Rotate(startPos);    

        return true;       
    }

    Xarm6Position XarmGrip::Realtest(Xarm6Position startPos, commonType::TargetReco target_msg)
    {
        Xarm6Position XarmGrip;
        XarmGrip.x = target_msg.x * 1000;
        XarmGrip.y = target_msg.y * 1000;
        XarmGrip.z = target_msg.z * 1000;
        XarmGrip.roll = M_PI;
        XarmGrip.pitch = -1.5707;
        XarmGrip.yaw = 0;
        
        //选找到目标角度
        float angle = XarmRotate::Instance().RotatetoTargetAngle(XarmGrip);
        XarmGrip.yaw = 0;//startPos.joint[0];
        return XarmGrip;
    }

    /// @brief 判断当前机械臂膀处于的状态 1（抓取），2（放置），3（回到home）
    /// @param control 
    /// @return 
    XarmGripState XarmGrip::CheckXarmCurrentState(commonType::XarmControlCommond& control)
    {
        if(control.GripperControlModel == 1)
        {
            return XarmGripState_Grip;
        }else if(control.GripperControlModel == 2)
        {
            return XarmGripState_UnGrip;
        }else if(control.GripperControlModel == 3)
        {
            return XarmGripState_Home;
        }else return XarmGripState_Null;
    }




} 