#ifndef COMMON_H_
#define COMMON_H_

#include "ros/ros.h"
#include "commonType/XarmStatus.h"
#include "commonType/TargetReco.h"
#include "commonType/XarmControlCommond.h"
#include "commonType/XarmGetTarget.h"
#include "xarm_msgs/SetInt16.h"
#include "xarm_msgs/Move.h"
#include "xarm_msgs/GripperMove.h"
#include "xarm_msgs/SetAxis.h"
#include "xarm_msgs/GripperConfig.h"
#include "xarm_msgs/GripperState.h"
#include "xarm_msgs/RobotMsg.h"
#include "xarm_msgs/GripperMove.h"
#include "xarm_msgs/ClearErr.h"
#include "unity_robotics_demo_msgs/APPGripMessage.h"
#include "unity_robotics_demo_msgs/APPConfirm.h"
#include "unity_robotics_demo_msgs/APPGripConfig.h"

namespace XARMGRIP_CONTROLLER{

    typedef  std::vector<commonType::TargetReco> TargetPoints;

    /// @brief 机械臂异常状态
    enum ErrorStatus{
        ErrorStatus_Normal,       //正常
        ErrorStatus_Error,         //异常
        ErrorStatus_Warning         //异常
    };
    /// @brief 机械臂控制指令
    enum ControlCommandStatus{
        ControlState_ALLOW,      //允许
        ControlState_REJECT      //拒绝
    };


    enum TargetStatus{
        TargetStatus_Error = 0,         //失败
        TargetStatus_Success = 1,       //成功
        TargetStatus_OverTime = -1       //超时
    };

    enum XarmGripState{
        XarmGripState_Null = 0,               //无控制指令
        XarmGripState_Grip = 1,               //抓取阶段
        XarmGripState_UnGrip = 2,             //释放
        XarmGripState_start = 3,              //开始阶段
        XarmGripState_Cycle = 4,               //巡检阶段
        XarmGripState_Home = 5,
        XarmGripState_Confirm = 6
    };

    enum APPXarmGripControlState{
        APPXarmGripControl_NULL,            //空
        APPXarmGripControl_GRIP,            //抓取
        APPXarmGripControl_CONFIRM          //标定
    };

    typedef struct{
        ros::Time GripControlTime;         //时间
        unity_robotics_demo_msgs::APPGripConfig GripControl;
        ros::Time ConfirmControlTime;         //时间
        unity_robotics_demo_msgs::APPConfirm ConfirmControl;
    }APPControl;

    //0(无任务)，1（抓取），2（放置），3（回到home）

    /// @brief 机械臂坐标
    class Xarm6Position{
        public:
            Xarm6Position(){
            };
            /// @brief 更新时间
            void timeUpdate(){
                Time = ros::Time::now();
            }
        public:
            float x;                //x坐标
            float y;                //y坐标
            float z;                //z坐标
            float roll;             //roll坐标
            float pitch;            //pitch坐标
            float yaw;              //yaw坐标
            ros::Time Time;         //时间
            float joint[6];         //关节
            float jointMaxVel;      //关节速度
            float jointAcc;         //关节加速度
            bool  jointControl = false;
    };
    /// @brief 机械爪坐标
    class GripPosition{
        public:
            GripPosition(){
                Time = ros::Time::now();
            }
            /// @brief 更新时间
            void timeUpdate(){
                Time = ros::Time::now();
            }
        public:
            float size;             //大小
            ros::Time Time;         //时间
    };
    /// @brief 机械臂和机械抓坐标
    class Xarm6AndGripControl
    {
        public:
            Xarm6AndGripControl(){
                xarm6Control = false;
                gripControl = false;
            }
            /// @brief 更新时间
            void timeUpdate(){
                Time = ros::Time::now();
            }
        public:
            Xarm6Position xarm6;
            bool xarm6Control;         //false 表示不控制，true表示控制 
            GripPosition grip;
            bool gripControl;          //false 表示不控制，true表示控制
            ros::Time Time;            //时间
    };

    class Param{
        public:
            //机械臂状态topic
            std::string xarm6LcationStatus_topic = "/xarm/xarm_states";
            //目标检测topic
            std::string xarm6TargetRecognition_topic = "/xarm/xarm_states";
            // //机械臂控制话题
            // std::string xarm6Control_topic = "/xarm/xarm_states";
            //机械臂关节使能服务名称
            std::string SetAxisServerName = "/xarm/motion_ctrl";
            //机械臂控制模式服务名称
            std::string SetModeServerName = "/xarm/set_mode";
            //机械臂状态服务名称
            std::string SetStateServerName = "/xarm/set_state";
            //机械臂控制服务名称
            std::string SetMoveLineServerName = "/xarm/move_line";
            //机械爪控制服务名称
            std::string SetGripperMoveServerName = "/xarm/gripper_move";
            //机械爪速度控制服务名称
            std::string SetGripperConfigServerName = "/xarm/gripper_config";
            //机械爪位置获取服务名称
            std::string SetGripperStateServerName = "/xarm/gripper_state";
            //机械臂清除异常的名称
            std::string SetClearXarmErrorName = "/xarm/clear_err";
            //机械臂角度控制的名字
            std::string SetXarm6ControlServerName = "/xarm/move_joint";
            //机械臂控制请求目标检测的名字
            std::string RosPubGetTargetTopic = "/xarm/GetTargetRec";
            //接受来自客户端的控制请求topic
            std::string Sub_Control_Topic = "/xarm/xarmClientControl";
            //参数
            int XarmControlModel = 0;          //模式
            int XarmRotateModel = 6;
            int XarmState = 0;          //状态
            int GripperConfig = 1500;   //机械爪速度

            float rotateDeltAngle;    //旋转单位角度
            float rotateMistake;      //旋转角度误差
            // int rotateSleepTime;      //旋转休眠 
            float controlMistake;      //控制误差
            Xarm6Position home;        //机械臂初始位置
            int RosFrequency;          //ros频率

            float Controlmvvelo;// : 200
            float Controlmvacc;// : 2000
            float Controlmvtime;// : 0
            float Controlmvradii;// : 0
            float grip_open;
            float grip_close;
            int OverTime;   //目标检测超时时间

            float Rotatemvvelo;// : 200
            float Rotatemvacc;// : 2000
            float Rotatemvtime;// : 0
            float Rotatemvradii;// : 0
    };




}
#endif