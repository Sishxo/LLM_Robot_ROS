#ifndef XARM_DIKER_CONTROLLER_H_
#define XARM_DIKER_CONTROLLER_H_

#include "common.h"
#include "ParamROS.h"
#include "XarmGrip_ROS.h"
#include <mutex>

namespace XARMGRIP_CONTROLLER
{
    class XarmDiKer
    {
        public:
            ~XarmDiKer(){};
            XarmDiKer(const XarmDiKer&) = delete;
            XarmDiKer& operator=(const XarmDiKer&) = delete;
            static XarmDiKer& Instance() {
                static XarmDiKer m_pInstance;
                return m_pInstance;
        
            }
        private:
            XarmDiKer(){
                
            };
            float GetDistance(Xarm6Position pos1, Xarm6Position pos2);

        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            /// @brief 根据目标点，采用笛卡尔坐标系控制
            /// @param pos 目标坐标
            /// @return true，表示到达目的地，false，表示异常
            bool DikerControl(Xarm6Position pos);
            /// @brief 退出
            void DiKerClose();

    };
} 


#endif