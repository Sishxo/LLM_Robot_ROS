#ifndef APPCLIENT_H__
#define APPCLIENT_H__

#include "common.h"
#include "XarmGrip_ROS.h"
#include "GripObject.h"
#include "Xarm_Rotate_Controller.h"
namespace XARMGRIP_CONTROLLER
{
    class APPClient
    {
        public:
            ~APPClient(){};
            APPClient(const APPClient&) = delete;
            APPClient& operator=(const APPClient&) = delete;
            static APPClient& Instance() {
                static APPClient m_pInstance;
                return m_pInstance;
        
            }
        private:
            APPClient(){};

        private:
            bool DoGrip(int objectID, commonType::TargetReco& target_msg);
            bool DoRelease(commonType::TargetReco Pos);
            Xarm6Position Realtest(Xarm6Position startPos, commonType::TargetReco target_msg);
        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            APPXarmGripControlState GetAPPControlCommand(APPControl& control);
            /// @brief APP无操作
            /// @param APPCommand 
            /// @return 
            int APPDONull(APPControl &APPCommand);
            /// @brief 抓取操作
            /// @param APPCommand 
            /// @return 
            int APPDoGrip(APPControl &APPCommand);
            /// @brief 标定操作
            /// @param APPCommand 
            /// @return 
            int APPDoConfirm(APPControl &APPCommand);

            void APPAddConfim(unity_robotics_demo_msgs::APPConfirm control);

            commonType::TargetReco APPGetTargetPos(char pos_label, int id);
    };
}

#endif