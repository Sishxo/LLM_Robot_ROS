#ifndef COMMON_H_
#define COMMON_H_

namespace Target_Recognition
{
    class ImageTarget
    {
    public:
        ImageTarget() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0),
                        sizeX(0), sizeY(0),
                        recognition_status(0),
                        mistake(0),
                        ObjectID(-1) {}

    public:
        /// @brief 坐标
        float x;
        float y;
        float z;
        float roll;
        float pitch;
        float yaw;
        /// @brief 物体大小,长-宽-高
        float sizeX;
        float sizeY;
        float sizeZ;
        /// @brief 识别状态
        int recognition_status;
        /// @brief 误差
        float mistake;
        /// @brief 识别物体ID
        float ObjectID;
    };

    class DetectionResult
    {
    public:
        DetectionResult() : x1(0), x2(0), y1(0), y2(0), detection_status(0), mistake(0), ObjectID(-1), ObjectName(NULL) {}

    public:
        /// @brief 目标检测bounding box结果
        float x1;
        float x2;
        float y1;
        float y2;
        /// @brief 目标检测状态
        int detection_status;
        /// @brief 误差
        float mistake;
        /// @brief 识别物体ID
        float ObjectID;
        /// @brief 识别物体名称
        std::string ObjectName;
    };

    class Param
    {
    public:
        std::string Rgb_Image_Topic = "/camera/color/image_raw";
        std::string Depth_Image_Topic = "/camera/aligned_depth_to_color/image_raw";
        std::string Xarm_Control_Topic = "/xarm/XarmControl";
        std::string Xarm_Status_Topic = "/xarm/XarmControlStatus";
        std::string Xarm_Recognition_Topic = "/xarm/XarmTargetRec";
        std::string Xarm_GetRecognition_Topic = "xarm/GetTargetRec";
        std::string Python_Module = "";
        std::string Python_Class_Name = "";
        // std::string
        int ROS_Frequency = 100;
        int Rgb_Depth_Delt_Time = 10;
        int Algorithm_ID = 1;
        int ImageDevice = 3;
    };

    enum XarmCallBackState
    {
        XarmCallBackState_IDLE,    // 空闲
        XarmCallBackState_WORKING, // 工作中
        XarmCallBackState_ERROR    // 异常
    };

    enum XarmControlState
    {
        XarmControlState_ALLOW, // 允许
        XarmControlState_REJECT // 拒绝
    };

    enum XarmErrorState
    {
        XarmErrorState_ERROR, // 异常
        XarmErrorState_NORMAL // 正常
    };
    /// 识别结果，0（失败）、1（成功）、-1（超时间）
    enum XarmRecognitionStatus
    {
        XarmRecognition_FAIL = 0,     // 异常
        XarmRecognition_SUCCESS = 1,  // 正常
        XarmRecognition_TIMEOUT = -1, // 超时
    };
    enum GripperControlState
    {
        GripperControl_IDLE = 0,
        GripperControl_GRAB = 1,
        GripperControl_PLACE = 2,
        GripperControl_HOME = 3
    };
}

#endif