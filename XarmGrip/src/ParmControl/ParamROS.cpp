#include "ParamROS.h"

namespace XARMGRIP_CONTROLLER
{
    void ParamROS::Init() {
        // 参数加载
        out.xarm6LcationStatus_topic = getParam<std::string>("/xarm6Grip_node/Rostopic/xarm6LcationStatus_topic","/xarm/xarm_states");
        out.xarm6TargetRecognition_topic = getParam<std::string>("/xarm6Grip_node/Rostopic/xarm6TargetRecognition_topic", "/xarm/xarm_states");
        // out.xarm6Control_topic = getParam<std::string>("/xarm6Grip_node/Rostopic/xarm6Control_topic", "/xarm/xarm_states");
        out.SetAxisServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetAxisServerName", "/xarm/motion_ctrl");
        out.SetModeServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetModeServerName",  "/xarm/set_mode");
        out.SetStateServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetStateServerName", "/xarm/set_state");
        out.SetMoveLineServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetMoveLineServerName", "/xarm/move_line");
        out.SetGripperMoveServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetGripperMoveServerName", "/xarm/gripper_move");
        out.SetGripperConfigServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetGripperConfigServerName", "/xarm/gripper_config");
        out.SetGripperStateServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetGripperStateServerName", "/xarm/gripper_state");
        out.SetClearXarmErrorName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetClearXarmErrorName", "/xarm/clear_err");
        out.SetXarm6ControlServerName = getParam<std::string>("/xarm6Grip_node/Rostopic/SetXarm6ControlServerName", "/xarm/move_joint");
        out.RosPubGetTargetTopic = getParam<std::string>("/xarm6Grip_node/Rostopic/RosPubGetTargetTopic", "/xarm/getTarget");
        out.RosFrequency = getParam<int>("/xarm6Grip_node/Rostopic/RosFrequency",100);

        out.XarmControlModel = getParam<int>("/xarm6Grip_node/XarmGripConfig/XarmControlModel",0);
        out.XarmRotateModel = getParam<int>("/xarm6Grip_node/XarmGripConfig/XarmRotateModel",6);
        out.XarmState = getParam<int>("/xarm6Grip_node/XarmGripConfig/XarmState",0);
        out.GripperConfig = getParam<int>("/xarm6Grip_node/XarmGripConfig/GripperConfig",1500);

        out.home.x = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomeX",100);
        out.home.y = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomeY",0);
        out.home.z = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomeZ",500);
        out.home.roll = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomeRoll",3.1415926);
        out.home.pitch = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomePitch",-1.5707);
        out.home.yaw = getParam<float>("/xarm6Grip_node/XarmControlConfig/HomeYaw",0);

        out.rotateDeltAngle = getParam<float>("/xarm6Grip_node/XarmControlConfig/XarmRotateDeltAngle",45);
        out.rotateMistake = getParam<float>("/xarm6Grip_node/XarmControlConfig/XarmRotateMistakeAngle",0);
        // out.rotateSleepTime = getParam<int>("/xarm6Grip_controller_node/XarmControlConfig/XarmRotateSleepTime",0);
        out.controlMistake = getParam<float>("/xarm6Grip_node/XarmControlConfig/XarmControlMistake",0);

        out.Controlmvvelo = getParam<float>("/xarm6Grip_node/XarmControlConfig/Controlmvvelo",200);
        out.Controlmvacc = getParam<float>("/xarm6Grip_node/XarmControlConfig/Controlmvacc",2000);
        out.Controlmvtime = getParam<int>("/xarm6Grip_node/XarmControlConfig/Controlmvtime",0);
        out.Controlmvradii = getParam<float>("/xarm6Grip_node/XarmControlConfig/Controlmvradii",0);

        out.Rotatemvvelo = getParam<float>("/xarm6Grip_node/XarmControlConfig/Rotatemvvelo",0);
        out.Rotatemvacc = getParam<float>("/xarm6Grip_node/XarmControlConfig/Rotatemvacc",0.1);
        out.Rotatemvtime = getParam<int>("/xarm6Grip_node/XarmControlConfig/Rotatemvtime",0);
        out.Rotatemvradii = getParam<float>("/xarm6Grip_node/XarmControlConfig/Rotatemvradii",7);

        out.grip_open = getParam<float>("/xarm6Grip_node/XarmGripConfig/Grip_open",850);
        out.grip_close = getParam<float>("/xarm6Grip_node/XarmGripConfig/Grip_close",0);
        out.OverTime = getParam<int>("/xarm6Grip_node/XarmControlConfig/OverTime",3);
        out.Sub_Control_Topic = getParam<std::string>("/xarm6Grip_node/Rostopic/Sub_Control_Topic", "/xarm/xarmClientControl");
    }

    Param ParamROS::GetParam()
    {
        return out;
    }

        //读取参数模板
    template<typename T> T ParamROS::getParam(const std::string& name,const T& defaultValue)
    {
        T v;
        if(ros::param::get(name,v))//get parameter by name depend on ROS.
        {
            // ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
            return v;
        }
        else 
            // ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
    }
} 