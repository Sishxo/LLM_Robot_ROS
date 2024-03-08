#include "Grip_Controller.h"

namespace XARMGRIP_CONTROLLER
{
    /// @brief 初始化
    void GripControl::Init()
    {
        GripOpen();
    }
    /// @brief 关闭
    void GripControl::Close()
    {

    }
    
    /// @brief 机械爪张开
    /// @return 
    bool GripControl::GripOpen()
    {
        int size = (int)ParamROS::Instance().GetParam().grip_open;
        int speed = (int)ParamROS::Instance().GetParam().GripperConfig;
        XarmGripROS::Instance().GripControlModelSet(speed);
        XarmGripROS::Instance().GripControl(size);
        
        return true;
    }
    /// @brief 机械抓抓取
    /// @param size 
    /// @return 
    bool GripControl::GripClose(int size)
    {
        XarmGripROS::Instance().GripControl(size);
        return true;
    }
}