#ifndef XARMGRIP_ROS_H_
#define XARMGRIP_ROS_H_

#include "common.h"
#include "ParamROS.h"
#include "commonType/XarmStatus.h"
#include "commonType/TargetReco.h"
#include "commonType/XarmControlCommond.h"
#include "commonType/XarmGetTarget.h"
#include <mutex>
#include <tf2_ros/transform_broadcaster.h> //创建TF广播器的功能包
#include <geometry_msgs/TransformStamped.h> //包含坐标转换的相关信息
#include <tf2/LinearMath/Quaternion.h> //欧拉角转换四元数功能包
#include <tf2_ros/buffer.h>//订阅的信息存储区创建功能包
#include <tf2_ros/transform_listener.h>//创建订阅方功能包
#include <geometry_msgs/PoseStamped.h>//坐标点表示功能包
#include <geometry_msgs/PointStamped.h>//坐标点表示功能包
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" 
#include "unity_robotics_demo_msgs/APPGripMessage.h"
#include "unity_robotics_demo_msgs/APPConfirm.h"
#include "unity_robotics_demo_msgs/APPGripConfig.h"
#include "ConfirmPoint.h"

namespace XARMGRIP_CONTROLLER
{
    class XarmGripROS
    {
        public:
            ~XarmGripROS();
            XarmGripROS(const XarmGripROS&) = delete;
            XarmGripROS& operator=(const XarmGripROS&) = delete;
            static XarmGripROS& Instance() {
                static XarmGripROS m_pInstance;
                return m_pInstance;
        
            }
        private:
            XarmGripROS(){};

        private:
            Param Paramdata;

            ros::ServiceClient rosSetSetAxisClient_;
            ros::ServiceClient rosSetModeClient_;
            ros::ServiceClient rosSetStateClient_;
            ros::ServiceClient rosGripperConfigClient_;
            ros::ServiceClient rosSetClearClient_;
            ros::ServiceClient rosJointControlClient_;
            
            ros::ServiceClient rosMoveLineClient_;
            ros::ServiceClient rosGripperMoveClient_;
            ros::ServiceClient rosGripperStateClient_;
            ros::Subscriber SubXarm6LcationStatus_;
            ros::Subscriber SubXarm6targetRecognition_;
            ros::Subscriber SubXarm6Control_;
            ros::Subscriber SubXarm6ClientControl_;

            ros::Publisher  PubGetTargetControl_;
            xarm_msgs::RobotMsg Xarm6RobotStatus_;      //机械臂状态
            commonType::TargetReco target_msg_;         //目标检测结果
            commonType::TargetReco controlTarget_msg_;  //目标检测结果
            commonType::XarmControlCommond control_msg_; //客户端控制请求

            // 3. 创建订阅对象: ---> 订阅坐标系相对关系 头文件2\3
            // 3.1 创建一个buffer缓存
            tf2_ros::Buffer buffer;
            // 3.2 再创建监听对象(监听对象可以将订阅的数据存入buffer)
            tf2_ros::TransformListener *listener;//(buffer);

            Xarm6Position xarm6Pos_;

            ros::Subscriber SubAPPGripConfig_;
            ros::Publisher  PubAPPGripMessage_;
            ros::Publisher  PubAPPConfirmMessage_;
            ros::Subscriber SubAPPConfirmConfig_;

            APPControl APPControl_;

        private:
            std::mutex xarm6Location_mutex;             //xarm6定位锁
            std::mutex targetRecognition_mutex;         //目标检查定位锁
            std::mutex xarm6Control_mutex;              //xarm6控制锁

        private:
            /// @brief 机械臂位置回调
            /// @param status_msg 
            void xarm6LocationCallBack(const xarm_msgs::RobotMsgConstPtr& status_msg);
            /// @brief 目标检测回调
            /// @param target_msg 
            void targetRecognitionCallBack(const commonType::TargetRecoConstPtr& target_msg);
            // /// @brief 机械臂控制回调
            // /// @param control_msg 
            // void xarm6ControlCallBack(const commonType::XarmControlCommondConstPtr& control_msg);
            /// @brief 机械臂客户端控制回调
            /// @param control_msg 
            void xarm6ClientControlCallBack(const commonType::XarmControlCommondConstPtr& control_msg);
            /// @brief 发送目标检测请求
            /// @param  
            void SendTargetRecoCommand(int objectID, commonType::XarmGetTarget& msg);

            bool XarmCameraBaseTF(commonType::TargetReco &msg);

            void XarmTFInit();
            /// @brief APP抓取回调检测函数
            /// @param confirm_msg 
            void APPGripConfigCallback(const unity_robotics_demo_msgs::APPGripConfigConstPtr& app_msg);
            /// @brief APP标定回调检测函数
            void APPConfirmConfigCallback(const unity_robotics_demo_msgs::APPConfirmConstPtr& confirm_msg);

        public:

            /// @brief 初始化
            void Init();
            ///机械爪速度控制
            bool GripControlModelSet(int speed);
            ///机械臂模式控制
            bool XarmControlModelSet(int Model);
            /// @brief 获取机械臂位置
            /// @return 
            Xarm6Position GetXarmLocation(void);
            /// @brief 获取机械爪的大小
            /// @param  
            /// @return 
            GripPosition GetGripLocation(void);
            /// @brief 获取目标检测结果
            /// @param  
            /// @return 
            // commonType::TargetReco GetTargetReco(void);
            bool GetTargetReco(commonType::TargetReco& msg);
            /// @brief 机械臂控制
            /// @param Model 
            /// @return 
            bool XarmControl(Xarm6Position control);
            /// @brief 机械爪控制
            /// @param  
            /// @return 
            bool GripControl(int size);
            /// @brief 获取目标识别的结果
            /// @param target_msg 结果
            /// @return true 获取成功，false 获取失败
            bool GetTargetRecognition(int objectID, commonType::TargetReco& target_msg);

            /// @brief 摄像头拍摄坐标转机械臂地卡尔坐标
            /// @param msg 
            bool XarmCameraTF(commonType::TargetReco &msg);
            /// @brief 获取客户端控制指令
            /// @param control 
            /// @return true表示接受到最新控制指令，false表示没有接收到控制指令
            bool GetClientControlCommand(commonType::XarmControlCommond& control);
            /// @brief 从手机app获取控制指令
            /// @param control 
            /// @return 
            APPXarmGripControlState GetAPPControlCommand(APPControl& control);
            /// @brief 发送定位结果
            /// @param target_msg 
            void APPConfirmPub(commonType::TargetReco& target_msg);





    };
} 


#endif