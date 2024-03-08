#ifndef GRIP_CONTROLLER_H_
#define GRIP_CONTROLLER_H_

#include "common.h"
#include "ParamROS.h"
#include "XarmGrip_ROS.h"

namespace XARMGRIP_CONTROLLER
{
    class GripControl
    {
        public:
            ~GripControl(){};
            GripControl(const GripControl&) = delete;
            GripControl& operator=(const GripControl&) = delete;
            static GripControl& Instance() {
                static GripControl m_pInstance;
                return m_pInstance;
        
            }
        private:
            GripControl(){
                
            };
        public:
            /// @brief 初始化
            void Init();
            /// @brief 关闭
            void Close();
            /// @brief 机械爪张开
            /// @return 
            bool GripOpen();
            /// @brief 机械抓抓取
            /// @param size 
            /// @return 
            bool GripClose(int size);
    };

}
#endif