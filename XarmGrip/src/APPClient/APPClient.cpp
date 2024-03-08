#include "APPClient.h"

namespace XARMGRIP_CONTROLLER
{
    /// @brief 初始化
    void APPClient::Init()
    {

    }
    /// @brief 关闭
    void APPClient::Close()
    {

    }
    /// @brief 从手机app获取控制指令
    /// @param control 
    /// @return 
    APPXarmGripControlState APPClient::GetAPPControlCommand(APPControl& control)
    {
        return XarmGripROS::Instance().GetAPPControlCommand(control);
    }
    /// @brief APP无操作
    /// @param APPCommand 
    /// @return 
    int APPClient::APPDONull(APPControl &APPCommand)
    {
        return 0;
    }
    /// @brief 抓取操作
    /// @param APPCommand 
    /// @return 
    int APPClient::APPDoGrip(APPControl &APPCommand)
    {
        XarmGripState state = XarmGripState_start;
        commonType::XarmControlCommond control;
        commonType::TargetReco target_msg;
        int objectID = APPCommand.GripControl.ObjectID;
        int GripPushID = APPCommand.GripControl.GripPushID;

        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);

        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep(); 
            switch (state)
            {
                //在当前位置获取目标检查结果，如果发现目标则进入控制模式，如果没有则进入巡检
                case XarmGripState_start:
                {
                    ROS_INFO("开始...............");
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID, target_msg) == true)
                    {
                        state = XarmGripState_Grip;
                    }else{
                        state = XarmGripState_Cycle;
                    }
                    break;
                }
                //巡检
                case XarmGripState_Cycle:  
                {
                    ROS_INFO("巡检...............");
                    Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();
                    if(XarmRotate::Instance().CycleRotate(startPos, objectID) == true)
                    {
                        ROS_INFO("巡检检测到目标...............");
                        state = XarmGripState_Grip;
                    }else{
                        ROS_INFO("巡检未检测到目标,回到home...............");
                        //回到原始位置 
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return -1;
                    }
                    break;    
                }   
                //抓取+释放
                case XarmGripState_Grip:
                {
                    ROS_INFO("先抓取，后放置...............");
                    commonType::TargetReco Pos;

                    // commonType::TargetReco target_msg;
                    if(DoGrip(objectID, Pos) == false) 
                    {
                        ROS_INFO("抓取异常，回到原位...............");
                        //回到原始位置 
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return -1;
                    }else{
                        ROS_INFO("抓取成功...............");
                    }
                    Pos = APPGetTargetPos((char)APPCommand.GripControl.EndingPos, GripPushID);
                    if(DoRelease(Pos) == false)
                    {
                        ROS_INFO("放置异常，回到原位...............");
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return -1;
                    }else{
                        ROS_INFO("放置成功...............");
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return 1;
                    }
                    break;  
                }
            }
        }

    }
    /// @brief 标定操作
    /// @param APPCommand 
    /// @return 
    int APPClient::APPDoConfirm(APPControl &APPCommand)
    {
        XarmGripState state = XarmGripState_start;
        commonType::XarmControlCommond control;
        commonType::TargetReco target_msg;
        int objectID = 0;
        ros::Rate loop_rate(ParamROS::Instance().GetParam().RosFrequency);

        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep(); 
            switch (state)
            {
                //在当前位置获取目标检查结果，如果发现目标则进入控制模式，如果没有则进入巡检
                case XarmGripState_start:
                {
                    ROS_INFO("开始...............");
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID, target_msg) == true)
                    {
                        state = XarmGripState_Confirm;
                    }else{
                        state = XarmGripState_Cycle;
                    }
                    break;
                }
                //巡检
                case XarmGripState_Cycle:  
                {
                    ROS_INFO("巡检...............");
                    Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();
                    if(XarmRotate::Instance().CycleRotate(startPos, objectID) == true)
                    {
                        ROS_INFO("巡检检测到目标...............");
                        state = XarmGripState_Confirm;
                    }else{
                        ROS_INFO("巡检未检测到目标,回到home...............");
                        //回到原始位置 
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return -1;
                    }
                    break;    
                }   
                //抓取+释放
                case XarmGripState_Confirm:
                {
                    ROS_INFO("标定.....");
                    commonType::TargetReco target_msg;
                    if(XarmGripROS::Instance().GetTargetRecognition(objectID,  target_msg) == true) 
                    {
                        XarmGripROS::Instance().XarmCameraTF(target_msg);
                        XarmGripROS::Instance().APPConfirmPub(target_msg);
                        //回到原始位置 
                        Xarm6Position home = GripObject::Instance().GoHome();
                        XarmRotate::Instance().Rotate(home);
                        return 1;
                    }else{
                        return -1;
                    }
                }
            }
        }

        return 0;
    }

    bool APPClient::DoGrip(int objectID, commonType::TargetReco& target_msg)
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

    bool APPClient::DoRelease(commonType::TargetReco Pos)
    {
        //获取起始坐标
        Xarm6Position startPos = XarmGripROS::Instance().GetXarmLocation();                    
        Xarm6Position targetPos = Realtest(startPos, Pos);
        //获取当前坐标
        Xarm6Position startPos1 = XarmGripROS::Instance().GetXarmLocation();
        //运动到object位置
        if(GripObject::Instance().Release(startPos1, targetPos) == false) return false;
        //抓取
        if(GripObject::Instance().UnGrip() == false) return false;
        //收回
        if(GripObject::Instance().Back() == false) return false;
        //回到最开始位置
        // XarmRotate::Instance().Rotate(startPos);    

        return true;       
    }

    Xarm6Position APPClient::Realtest(Xarm6Position startPos, commonType::TargetReco target_msg)
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

    void APPClient::APPAddConfim(unity_robotics_demo_msgs::APPConfirm control)
    {

    }

    commonType::TargetReco APPClient::APPGetTargetPos(char pos_label, int id)
    {
        std::string label = "";
        label += pos_label;
        std::vector<commonType::TargetReco> Points;
        ConfirmPoint::Instance().GetYamlPoints(label, Points);
        int tempID = id % Points.size();
        // std::cout<<Points[0].x<<"  "<<Points[0].y<<"  "<<Points[0].z<<std::endl;
        // XarmGripROS::Instance().GetTargetRecognition(objectID, target_msg) ;
        // XarmGripROS::Instance().XarmCameraTF(Points[0]);
        return Points[tempID];
    }
}