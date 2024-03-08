#include "XarmGrip_ROS.h"
#define GRIP_USE 1

namespace XARMGRIP_CONTROLLER
{

    void XarmGripROS::Init()
    {
        ros::Time::init();
        Paramdata = ParamROS::Instance().GetParam();
        ros::NodeHandle Node_ = ParamROS::Instance().GetRosNode();

        //ros关节配置 机械臂关节使能服务名称
        ros::service::waitForService(Paramdata.SetAxisServerName);
	    rosSetSetAxisClient_ = Node_.serviceClient<xarm_msgs::SetAxis>(Paramdata.SetAxisServerName); 
        //ros模式配置 机械臂控制模式服务名称
        ros::service::waitForService(Paramdata.SetModeServerName);
	    rosSetModeClient_ = Node_.serviceClient<xarm_msgs::SetInt16>(Paramdata.SetModeServerName); 
        //ros状态配置 机械臂状态服务名称
        ros::service::waitForService(Paramdata.SetStateServerName);
	    rosSetStateClient_ = Node_.serviceClient<xarm_msgs::SetInt16>(Paramdata.SetStateServerName); 
        //ros机械臂控制 机械臂控制服务名称
        ros::service::waitForService(Paramdata.SetMoveLineServerName);
	    rosMoveLineClient_ = Node_.serviceClient<xarm_msgs::Move>(Paramdata.SetMoveLineServerName); 

        
#if GRIP_USE
        //ros机械爪控制 机械爪控制服务名称
        ros::service::waitForService(Paramdata.SetGripperMoveServerName);
	    rosGripperMoveClient_ = Node_.serviceClient<xarm_msgs::GripperMove>(Paramdata.SetGripperMoveServerName); 
        //ros机械抓速度控制 机械爪速度控制服务名称
        ros::service::waitForService(Paramdata.SetGripperConfigServerName);
	    rosGripperConfigClient_ = Node_.serviceClient<xarm_msgs::GripperConfig>(Paramdata.SetGripperConfigServerName); 
        //ros机械抓状态获取 机械爪位置获取服务名称
        ros::service::waitForService(Paramdata.SetGripperStateServerName);
	    rosGripperStateClient_ = Node_.serviceClient<xarm_msgs::GripperState>(Paramdata.SetGripperStateServerName); 
#endif
        //机械臂清除异常的名称
        ros::service::waitForService(Paramdata.SetClearXarmErrorName);
	    rosSetClearClient_ = Node_.serviceClient<xarm_msgs::ClearErr>(Paramdata.SetClearXarmErrorName); 
        //机械臂角度控制的名字
        ros::service::waitForService(Paramdata.SetXarm6ControlServerName);
        rosJointControlClient_ = Node_.serviceClient<xarm_msgs::Move>(Paramdata.SetXarm6ControlServerName);

        //机械臂位置回调 机械臂状态topic
        SubXarm6LcationStatus_ = Node_.subscribe(Paramdata.xarm6LcationStatus_topic,
                                        1,
                                        &XarmGripROS::xarm6LocationCallBack, 
                                        this); 
        //目标检测回调 目标检测topic
        SubXarm6targetRecognition_ = Node_.subscribe(Paramdata.xarm6TargetRecognition_topic,
                                        1,
                                        &XarmGripROS::targetRecognitionCallBack, 
                                        this);
        //控制回调 机械臂控制话题
        SubXarm6ClientControl_ = Node_.subscribe(Paramdata.Sub_Control_Topic,
                                        1,
                                        &XarmGripROS::xarm6ClientControlCallBack, 
                                        this);
        //请求目标检测 机械臂控制请求目标检测的名字
        PubGetTargetControl_ = Node_.advertise<commonType::XarmGetTarget>(Paramdata.RosPubGetTargetTopic, 1);

        SubAPPGripConfig_ = Node_.subscribe("/xarm/APPGripConfig", 
                                        1, 
                                        &XarmGripROS::APPGripConfigCallback, 
                                        this);
        PubAPPGripMessage_ = Node_.advertise<unity_robotics_demo_msgs::APPGripMessage>("/xarm/APPGripMessage", 1);

        PubAPPConfirmMessage_ = Node_.advertise<unity_robotics_demo_msgs::APPConfirm>("/xarm/APPConfirmMessage", 1);

        SubAPPConfirmConfig_ = Node_.subscribe("/xarm/APPConfirmConfig", 
                                        1, 
                                        &XarmGripROS::APPConfirmConfigCallback, 
                                        this);

        //tf监听初始化
        XarmTFInit();
    }
        
    XarmGripROS::~XarmGripROS()
    {

    }

    /// @brief 机械臂位置回调
    /// @param status_msg 
    void XarmGripROS::xarm6LocationCallBack(const xarm_msgs::RobotMsgConstPtr& status_msg)
    {
        xarm6Location_mutex.lock();
        Xarm6RobotStatus_  = *status_msg;
        Xarm6Position pos;
        pos.x     = status_msg->pose[0];                //x坐标
        pos.y     = status_msg->pose[1];                //y坐标
        pos.z     = status_msg->pose[2];                //z坐标
        pos.roll  = status_msg->pose[3];             //roll坐标
        pos.pitch = status_msg->pose[4];            //pitch坐标
        pos.yaw   = status_msg->pose[5];              //yaw坐标
        // std::cout<<pos.x<<"  "<<pos.y<<"  "<<pos.z<<std::endl;
        // std::cout<<pos.roll<<"  "<<pos.pitch<<"  "<<pos.yaw<<std::endl;
        
        pos.Time  = status_msg->header.stamp;         //时间
        pos.joint[0] = status_msg->angle[0];         //关节
        pos.joint[1] = status_msg->angle[1];         //关节
        pos.joint[2] = status_msg->angle[2];         //关节
        pos.joint[3] = status_msg->angle[3];         //关节
        pos.joint[4] = status_msg->angle[4];         //关节
        pos.joint[5] = status_msg->angle[5];         //关节
        // std::cout<<pos.joint[0]<<"  "<<pos.joint[5]<<std::endl;
        // std::cout<<std::endl;
        xarm6Pos_ = pos;
        xarm6Location_mutex.unlock();     
    }

    /// @brief APP抓取回调检测函数
    /// @param confirm_msg 
    void XarmGripROS::APPGripConfigCallback(const unity_robotics_demo_msgs::APPGripConfigConstPtr& app_msg)
    {
        APPControl_.GripControlTime = ros::Time::now();
        APPControl_.GripControl.StartingPos  = app_msg->StartingPos;
        APPControl_.GripControl.EndingPos    = app_msg->EndingPos;
        APPControl_.GripControl.ObjectID     = app_msg->ObjectID;
        APPControl_.GripControl.SpeedLevel   = app_msg->SpeedLevel;
        APPControl_.GripControl.Model        = app_msg->Model;
        APPControl_.GripControl.GripPushID   = app_msg->GripPushID;
        char buffer[100];
        sprintf(buffer, "rec order: obj%d speed%d from %c to %c\0", app_msg->ObjectID, app_msg->SpeedLevel,app_msg->StartingPos, app_msg->EndingPos);
        unity_robotics_demo_msgs::APPGripMessage msg;
        // msg.data = buffer;
        PubAPPGripMessage_.publish(msg);
    }
    /// @brief APP标定回调检测函数
    void XarmGripROS::APPConfirmConfigCallback(const unity_robotics_demo_msgs::APPConfirmConstPtr& confirm_msg)
    {
        if(confirm_msg->clear == true)
        {
            ROS_INFO("清除标定点");
            ConfirmPoint::Instance().ClearYamlPoint("A");
            ConfirmPoint::Instance().ClearYamlPoint("B");
            unity_robotics_demo_msgs::APPConfirm temp;
            temp.A_size = ConfirmPoint::Instance().GetYamlPointSize("A");
            temp.B_size = ConfirmPoint::Instance().GetYamlPointSize("B");
            PubAPPConfirmMessage_.publish(temp);
            return;
        }
        if(confirm_msg->Start == true){
            ROS_INFO("开始标定");
            APPControl_.ConfirmControlTime = ros::Time::now();
            unity_robotics_demo_msgs::APPConfirm temp;
            temp.A_size = ConfirmPoint::Instance().GetYamlPointSize("A");
            temp.B_size = ConfirmPoint::Instance().GetYamlPointSize("B");
            PubAPPConfirmMessage_.publish(temp);
        } 
        if(confirm_msg->Add == true) {
            ROS_INFO("增加标定点");
            APPControl_.ConfirmControl.Start = confirm_msg->Start;
            APPControl_.ConfirmControl.Add = confirm_msg->Add;
            APPControl_.ConfirmControl.ConfirmPos = confirm_msg->ConfirmPos;
            APPControl_.ConfirmControl.x = confirm_msg->x;
            APPControl_.ConfirmControl.y = confirm_msg->y;
            APPControl_.ConfirmControl.z = confirm_msg->z;
            APPControl_.ConfirmControl.roll = confirm_msg->roll;
            APPControl_.ConfirmControl.pitch = confirm_msg->pitch;
            APPControl_.ConfirmControl.yaw = confirm_msg->yaw;
            APPControl_.ConfirmControl.A_size = confirm_msg->A_size;
            APPControl_.ConfirmControl.B_size = confirm_msg->B_size;
            
            
            if(confirm_msg->ConfirmPos == 'A')
            {
                ROS_INFO("增加A标定点");
                std::string label = "";
                label += (char)confirm_msg->ConfirmPos;
                int id = confirm_msg->A_size;
                commonType::TargetReco points;
                points.x = confirm_msg->x;
                points.y = confirm_msg->y;
                points.z = confirm_msg->z;
                ConfirmPoint::Instance().SetYamlPoint(label, id, points);
                
            }else if(confirm_msg->ConfirmPos == 'B')
            {
                ROS_INFO("增加B标定点");
                std::string label = "";
                label += (char)confirm_msg->ConfirmPos;
                int id = confirm_msg->B_size;
                commonType::TargetReco points;
                points.x = confirm_msg->x;
                points.y = confirm_msg->y;
                points.z = confirm_msg->z;
                ConfirmPoint::Instance().SetYamlPoint(label, id, points);
            }
            unity_robotics_demo_msgs::APPConfirm temp;
            temp.A_size = ConfirmPoint::Instance().GetYamlPointSize("A");
            temp.B_size = ConfirmPoint::Instance().GetYamlPointSize("B");
            PubAPPConfirmMessage_.publish(temp);
            
            
        }
    }

    /// @brief 目标检测回调
    /// @param target_msg 
    void XarmGripROS::targetRecognitionCallBack(const commonType::TargetRecoConstPtr& target_msg)
    {
        targetRecognition_mutex.lock();
        controlTarget_msg_ = *target_msg;
        targetRecognition_mutex.unlock();
    }

    // /// @brief 机械臂控制回调
    // /// @param control_msg 
    // void XarmGripROS::xarm6ControlCallBack(const commonType::XarmControlCommondConstPtr& control_msg)
    // {
    //     xarm6Control_mutex.lock();
    //     control_msg_ = *control_msg;
    //     xarm6Control_mutex.unlock();
    // }

    /// @brief 机械臂客户端控制回调
    /// @param control_msg 
    void XarmGripROS::xarm6ClientControlCallBack(const commonType::XarmControlCommondConstPtr& control_msg)
    {

        ROS_INFO("接收到控制命令...............");
        xarm6Control_mutex.lock();
        control_msg_.stamp               = control_msg->stamp;
        control_msg_.TaskID              = control_msg->TaskID;
        control_msg_.ObjectID            = control_msg->ObjectID;
        control_msg_.GripperControlModel = control_msg->GripperControlModel;
        control_msg_.XarmControlModel    = control_msg->XarmControlModel;
        control_msg_.XarmSpeedLevel      = control_msg->XarmSpeedLevel;
        control_msg_.GripSpeedLevel      = control_msg->GripSpeedLevel;

        control_msg_.target_x            = control_msg->target_x;
        control_msg_.target_y            = control_msg->target_y;
        control_msg_.target_z            = control_msg->target_z;
        control_msg_.target_roll         = control_msg->target_roll;
        control_msg_.target_pitch        = control_msg->target_pitch;
        control_msg_.target_yaw          = control_msg->target_yaw;       
        xarm6Control_mutex.unlock();
    }

    /// @brief 从手机app获取控制指令
    /// @param control 
    /// @return 
    APPXarmGripControlState XarmGripROS::GetAPPControlCommand(APPControl& control)
    {
        if((APPControl_.GripControlTime != control.GripControlTime) || 
            (APPControl_.ConfirmControlTime != control.ConfirmControlTime))
        {
            if(APPControl_.GripControlTime != control.GripControlTime)
            {              
                control =  APPControl_;
                return APPXarmGripControl_GRIP;
            }else if(APPControl_.ConfirmControlTime != control.ConfirmControlTime)
            {
                control =  APPControl_;
                return APPXarmGripControl_CONFIRM;
            }
        }else{
            return APPXarmGripControl_NULL;
        }
        
    }

    bool XarmGripROS::GetClientControlCommand(commonType::XarmControlCommond& control)
    {
        bool flag = false;
        xarm6Control_mutex.lock();
        
        if(control.stamp == control_msg_.stamp)
        {
            flag = false;
        }else{
            flag = true;
        }
        control = control_msg_;
        
        xarm6Control_mutex.unlock();

        return flag;
    }

    ///机械爪速度控制
    bool XarmGripROS::GripControlModelSet(int speed)
    {
        //机械抓速度控制
        xarm_msgs::GripperConfig gripperConfig_srv;
        gripperConfig_srv.request.pulse_vel = speed;
        rosGripperConfigClient_.call(gripperConfig_srv);

        if(gripperConfig_srv.response.ret == 0) return true;
        else return false;
    }

    bool XarmGripROS::XarmControlModelSet(int Model)
    {
        //参数
        int XarmModel = Model;                  //模式
        int XarmState = Paramdata.XarmState;    //状态

        //清除异常
        xarm_msgs::ClearErr clear_srv;
        clear_srv.request;
        rosSetClearClient_.call(clear_srv);
        //参数初始化
        //机械臂状态回馈异常初始化为正常
        Xarm6RobotStatus_.err = 0;   
        Xarm6RobotStatus_.warn = 0;
        //机械臂关节使能
        xarm_msgs::SetAxis axis_srv;
        axis_srv.request.id = 8;
        axis_srv.request.data = 1;
        rosSetSetAxisClient_.call(axis_srv);
        //机械臂模式控制
        xarm_msgs::SetInt16 model_srv;
        model_srv.request.data = XarmModel;
        rosSetModeClient_.call(model_srv);
        //机械臂状态设置
        xarm_msgs::SetInt16 state_srv;
        state_srv.request.data = XarmState;
        rosSetStateClient_.call(state_srv);

        if((model_srv.response.ret == 0) && (state_srv.response.ret == 0)
         && (axis_srv.response.ret == 0)) return true;
        else return false;  
    }

    /// @brief 获取机械臂位置
    /// @return 
    Xarm6Position XarmGripROS::GetXarmLocation(void)
    {
        Xarm6Position out;
        xarm6Location_mutex.lock();
        out = xarm6Pos_;
        xarm6Location_mutex.unlock();


        return out;
    }

    /// @brief 获取机械爪的大小
    /// @param  
    /// @return 
    GripPosition XarmGripROS::GetGripLocation(void)
    {
        GripPosition local_msg;
        //获取机械爪大小 xarm_msgs/GripperState
        xarm_msgs::GripperState gripperState;
        rosGripperStateClient_.call(gripperState);
        local_msg.size = gripperState.response.curr_pos;  //机械爪赋值
        local_msg.timeUpdate();
        return local_msg;
    }

    /// @brief 获取目标检测结果
    /// @param  
    /// @return 
    bool XarmGripROS::GetTargetReco(commonType::TargetReco& msg)
    {

        targetRecognition_mutex.lock();
        msg = controlTarget_msg_;
        targetRecognition_mutex.unlock();
        return true;
    }

    /// @brief 机械臂控制
    /// @param Model 
    /// @return 
    bool XarmGripROS::XarmControl(Xarm6Position control)
    {
        if(control.jointControl == true){            
            //机械臂状态设置
            xarm_msgs::Move xarm;
            xarm.request.pose.resize(6);
            xarm.request.pose[0] = control.joint[0];
            xarm.request.pose[1] = control.joint[1];
            xarm.request.pose[2] = control.joint[2];
            xarm.request.pose[3] = control.joint[3];
            xarm.request.pose[4] = control.joint[4];
            xarm.request.pose[5] = control.joint[5];
            xarm.request.mvacc =    Paramdata.Rotatemvacc;//0.1;
            xarm.request.mvradii =  Paramdata.Rotatemvradii;//7;
            xarm.request.mvtime =   Paramdata.Rotatemvtime;
            xarm.request.mvvelo =   Paramdata.Rotatemvvelo;
            rosJointControlClient_.call(xarm);
        }else{
            //机械臂状态设置
            xarm_msgs::Move xarm;
            xarm.request.pose.resize(6);
            xarm.request.pose[0] = control.x;
            xarm.request.pose[1] = control.y;
            xarm.request.pose[2] = control.z;
            xarm.request.pose[3] = control.roll;
            xarm.request.pose[4] = control.pitch;
            xarm.request.pose[5] = control.yaw;
            xarm.request.mvacc = Paramdata.Controlmvacc;//Controlmvacc_;
            xarm.request.mvradii = Paramdata.Controlmvradii;//Controlmvradii_;
            xarm.request.mvtime = Paramdata.Controlmvtime;//Controlmvtime_;
            xarm.request.mvvelo = Paramdata.Controlmvvelo;//Controlmvvelo_;
            rosMoveLineClient_.call(xarm);
        }
        return true;
    }

    /// @brief 机械爪控制
    /// @param  
    /// @return 
    bool XarmGripROS::GripControl(int size)
    {
        xarm_msgs::GripperMove grip;
        grip.request.pulse_pos = size;
        rosGripperMoveClient_.call(grip);
        return true;
    }

    /// @brief 发送目标检测请求
    /// @param  
    void XarmGripROS::SendTargetRecoCommand(int objectID, commonType::XarmGetTarget& msg)
    {
        msg.stamp = ros::Time::now();
        msg.flag = true;
        msg.ObjectID = objectID;
        msg.overTime = Paramdata.OverTime;
        PubGetTargetControl_.publish(msg);
    }

    /// @brief 获取目标识别的结果
    /// @param target_msg 结果
    /// @return true 获取成功，false 获取失败
    bool XarmGripROS::GetTargetRecognition(int objectID, commonType::TargetReco& target_msg)
    {
        ros::Rate loop_rate(Paramdata.RosFrequency);
        //发送请求数据
        commonType::XarmGetTarget msg;
        SendTargetRecoCommand(objectID, msg);
        // 等待图像检测结果
        while(ros::ok())
        {
            commonType::TargetReco tempTarget;
            GetTargetReco(tempTarget);

            // if(GetTargetReco(target_msg) == false)

            //  判断接受到目标检测结果中的请求控制指令的时间与机械臂请求时间相同，且图像识别结果正确，返回true
            if((tempTarget.ControlStamp == msg.stamp) && (tempTarget.recognition_status == TargetStatus_Success))
            {
                
                ROS_INFO("接收到目标检测结果,检测成功");
                target_msg = tempTarget;
                return true;
            }
            //  判断接受到目标检测结果中的请求控制指令的时间与机械臂请求时间相同，图像识别结果异常，返回false
            else if((tempTarget.ControlStamp == msg.stamp) && (tempTarget.recognition_status != TargetStatus_Success))
            {
                // ROS_INFO("接收到目标检测结果,检测失败");
                return false;
            }

            // ros::Duration deltTime = ros::Time::now() - msg.stamp;
            // if(deltTime.toSec() >= Paramdata.OverTime)
            // {
            //     ROS_INFO("目标检测超时 %d s", Paramdata.OverTime);
            //     return false;
            // }
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

    bool XarmGripROS::XarmCameraTF(commonType::TargetReco &msg)
    {
        if(XarmCameraBaseTF(msg) == false)
        {
            if(XarmCameraBaseTF(msg) == false) return false;
        }
        return true;
    }

    /// @brief 摄像头拍摄坐标转机械臂地卡尔坐标
    /// @param msg 
    bool XarmGripROS::XarmCameraBaseTF(commonType::TargetReco &msg)
    {
        // 4. 组织一个坐标点数据 头文件4
        geometry_msgs::PointStamped ps;
        // 参考的坐标系
        ps.header.frame_id = "camera_color_optical_frame";// 坐标系 物体位置相对于乌龟坐标系而言
        // 时间戳
        ps.header.stamp = ros::Time(0.0);

        
        ps.point.x = msg.x;// * 1000; // 物体相对于乌龟坐标系的位置
        ps.point.y = msg.y;// * 1000 * (-1);
        ps.point.z = msg.z - 0.145;// * 1000 * (-1);
        // ROS_INFO("%f, %f, %f", ps.point.x, ps.point.y, ps.point.z);
        try
        {
            geometry_msgs::PointStamped ps_out; 
            ps_out = buffer.transform(ps,"link_base"); // 目标点信息，基参考系，输出转换后的坐标 头文件5r

            msg.x = ps_out.point.x;
            msg.y = ps_out.point.y;
            msg.z = ps_out.point.z;

            // 6. 最后输出 
            // ROS_INFO("转换后的坐标值:(%.2f,%.2f,%.2f),参考的坐标系:%s",
            //             ps_out.point.x,
            //             ps_out.point.y,
            //             ps_out.point.z,
            //             ps.header.frame_id.c_str()
            //             );
            return true;
        }
        catch(const std::exception& e)
        {
           ROS_INFO("异常消息:%s",e.what());
           return false;
        }
    }

    void XarmGripROS::XarmTFInit()
    {
        listener = new tf2_ros::TransformListener(buffer);
    }

    /// @brief 发送定位结果
    /// @param target_msg 
    void XarmGripROS::APPConfirmPub(commonType::TargetReco& target_msg)
    {
        unity_robotics_demo_msgs::APPConfirm msg;
        msg.x = target_msg.x;
        msg.y = target_msg.y;
        msg.z = target_msg.z;
        // msg.A_size = 1;
        // 
        std::vector<commonType::TargetReco> Points;
        ConfirmPoint::Instance().GetYamlPoints("A", Points);
        msg.A_size = Points.size();
        ConfirmPoint::Instance().GetYamlPoints("B", Points);
        msg.B_size = Points.size();

        PubAPPConfirmMessage_.publish(msg);
    }


} 