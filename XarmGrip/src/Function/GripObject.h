#ifndef GRIPOBJECT_H_
#define GRIPOBJECT_H_

#include "common.h"
#include "Xarm_DiKer_Controller.h"
#include "Grip_Controller.h"
#include "Xarm_Rotate_Controller.h"
namespace XARMGRIP_CONTROLLER
{
    class GripObject
    {
        public:
            ~GripObject(){};
            GripObject(const GripObject&) = delete;
            GripObject& operator=(const GripObject&) = delete;
            static GripObject& Instance() {
                static GripObject m_pInstance;
                return m_pInstance;
        
            }
        private:
            GripObject(){
                // ROS_INFO("GripObject init..........");
            };
            /// @brief 开始抓取
            /// @param objectPos 
            bool StartGrip();
            /// @brief 前往目标点
            /// @return 
            bool GoGrip();

            bool GoRelease();
            /// @brief 从目标点回
            /// @return 
            bool BackGrip();
            /// @brief 
            /// @return 
            bool EndGrip();

            Xarm6Position GetPrefab(Xarm6Position xarm6, Xarm6Position target);

            float GetDistance(Xarm6Position Target, Xarm6Position Xarm);

            Xarm6Position Realtest(commonType::TargetReco target_msg);

        private:
            Xarm6Position startPos_; 
            Xarm6Position target_;
            Xarm6Position prefabPos_;
        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            /// @brief 前往目标点
            /// @param objectPos 
            /// @return 
            bool Go(Xarm6Position startPos, Xarm6Position target);

            bool Release(Xarm6Position startPos, Xarm6Position target);
            /// @brief 抓取
            /// @return 
            bool Grip();
            /// @brief 放置
            /// @return 
            bool UnGrip();
            /// @brief 回到初始位置
            /// @return 
            bool Back();
            /// @brief 回到坐标原点
            Xarm6Position GoHome();

    };

}
#endif