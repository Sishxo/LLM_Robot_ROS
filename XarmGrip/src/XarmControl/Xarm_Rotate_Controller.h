#ifndef XARM_ROTATE_CONTROLLER_H_
#define XARM_ROTATE_CONTROLLER_H_

#include "common.h"
#include "ParamROS.h"
#include "XarmGrip_ROS.h"
#include <mutex>

namespace XARMGRIP_CONTROLLER
{
    class XarmRotate
    {
        public:
            ~XarmRotate(){};
            XarmRotate(const XarmRotate&) = delete;
            XarmRotate& operator=(const XarmRotate&) = delete;
            static XarmRotate& Instance() {
                static XarmRotate m_pInstance;
                return m_pInstance;
        
            }
        private:
            XarmRotate(){
                // ROS_INFO("XarmRotate init..........");
            };
            /// @brief 判断当前位置与目标位置的角度是否在阈值以内
            /// @param pos1 
            /// @param pos2 
            /// @param mistake 
            /// @return 
            bool CheckAngle(Xarm6Position pos1, Xarm6Position pos2, float mistake);
            /// @brief 计算pos1到pos2所在位置需要旋转多少角度
            /// @param Pos1 
            /// @param Pos2 
            /// @return 
            float GetDeltAngle(Xarm6Position Pos1, Xarm6Position Pos2);

        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            /// @brief 周期巡检
            /// @param startPos 起始位置 
            /// @param objectID 目标检测物体id
            /// @return true，表示目标检测成功，false，表示目标检测失败
            bool CycleRotate(Xarm6Position startPos, int objectID);
            /// @brief 角度旋转
            /// @param currentPos 当前位置
            /// @param angle 旋转角度
            /// @return true表示旋转成功，false表示旋转失败
            bool AngleRotate(Xarm6Position &currentPos, float angle);
            /// @brief 旋转到目标位置
            /// @param targetPos 
            /// @return 
            bool Rotate(Xarm6Position targetPos);
            /// @brief 旋转到目标位置所在角度
            /// @param targetPos 
            /// @return 
            bool RotatetoTargetAngle(Xarm6Position targetPos);


    };
} 


#endif