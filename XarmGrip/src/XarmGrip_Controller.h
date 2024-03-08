#ifndef XARMGRIP_CONTROLLER_H_
#define XARMGRIP_CONTROLLER_H_

#include "common.h"
#include "XarmGrip_ROS.h"
#include "Grip_Controller.h"
#include "ParamROS.h"
#include "Xarm_DiKer_Controller.h"
#include "Xarm_Rotate_Controller.h"
#include "GripObject.h"
#include "APPClient.h"
#include "ConfirmPoint.h"

namespace XARMGRIP_CONTROLLER
{
    class XarmGrip
    {
        public:
            XarmGrip();
            ~XarmGrip();
            /// @brief 初始化
            void Init();
            /// @brief 开始
            void Start();
        private:
            Xarm6Position Realtest(Xarm6Position startPos, commonType::TargetReco target_msg);
            /// @brief 判断当前机械臂膀处于的状态
            /// @param control 
            /// @return 
            XarmGripState CheckXarmCurrentState(commonType::XarmControlCommond& control);    
            /// @brief 抓
            /// @return 
            bool DoGrip(int objectID, commonType::TargetReco& target_msg);
            /// @brief 放
            /// @return 
            bool DoRelease(commonType::TargetReco Pos);
            // /// @brief APP无操作
            // /// @param APPCommand 
            // /// @return 
            // int APPDONull(APPControl &APPCommand);
            // /// @brief 抓取操作
            // /// @param APPCommand 
            // /// @return 
            // int APPDoGrip(APPControl &APPCommand);
            // /// @brief 标定操作
            // /// @param APPCommand 
            // /// @return 
            // int APPDoConfirm(APPControl &APPCommand);
    };
} 



#endif