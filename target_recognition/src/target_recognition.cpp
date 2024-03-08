#include "target_recognition.h"

#define USE_message_filters 1



namespace Target_Recognition
{

    ROS_Target_Recognition::ROS_Target_Recognition()
    {
        // 设置ROS_INFO输出中文
        setlocale(LC_CTYPE, "zh_CN.utf8");
        setlocale(LC_ALL, "");

        //参数加载
        Param rosParam = ROS_Target_Recognition::LoadPararms();

        ROS_INFO("当目标检测算法开始...............");

        // 算法初始化
        Algorithm_ = ROS_Target_Recognition::loadAlgorithm(rosParam);
        ROS_INFO("当前使用的算法为: [%s]", Algorithm_->Name().c_str());
        Algorithm_->Init();

        // ros接受和发送配置
        // SubXarmControl_          = Node_.subscribe(rosParam.Xarm_Control_Topic,
        //                                 1,
        //                                 &ROS_Target_Recognition::xarmControlCommandCallback, 
        //                                 this); 
        // SubXarmControlStatus_    = Node_.subscribe(rosParam.Xarm_Status_Topic,
        //                                 1,
        //                                 &ROS_Target_Recognition::xarmControlStatusCallback, 
        //                                 this); 

        SubXarmGetTargetRec_     = Node_.subscribe(rosParam.Xarm_GetRecognition_Topic,
                                        1,
                                        &ROS_Target_Recognition::xarmGetTargetRecCallback, 
                                        this);        


        PubTargetRecognition_    = Node_.advertise<commonType::TargetReco>(rosParam.Xarm_Recognition_Topic, 1);

#if USE_message_filters
        ROS_INFO("当前RGB-topic为: [%s]", rosParam.Rgb_Image_Topic.c_str());
        ROS_INFO("当前DEP-topic为: [%s]", rosParam.Depth_Image_Topic.c_str());
        ROS_INFO("当前DEP&RGB-deltTimec为: %d", rosParam.Rgb_Depth_Delt_Time);

        rgb_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(Node_, rosParam.Rgb_Image_Topic, 1);
        depth_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(Node_, rosParam.Depth_Image_Topic, 1);       
        sync_ = new  message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(rosParam.Rgb_Depth_Delt_Time), *rgb_sub_, *depth_sub_);
        sync_->registerCallback(boost::bind(&ROS_Target_Recognition::imageRealsenseCallback,this, _1, _2));

#else
        SubImageRgbRealsense_    = Node_.subscribe(rosParam.Rgb_Image_Topic, 
                                        1, 
                                        &ROS_Target_Recognition::imageRgbRealsenseCallback, 
                                        this);

        SubImageDepthRealsense_  = Node_.subscribe(rosParam.Depth_Image_Topic, 
                                        1, 
                                        &ROS_Target_Recognition::imageDepthRealsenseCallback, 
                                        this);
#endif
        //ros频率配置
        RosFrequency_ = rosParam.ROS_Frequency;
        //摄像头检查异常时间
        ImageDevice_ = rosParam.ImageDevice * 1.0 / 1000;
        XarmGetTargetRec_.flag = false;
        ROS_INFO("当前摄像头检查异常时间为: %f s", ImageDevice_);
    }

    ROS_Target_Recognition::~ROS_Target_Recognition()
    {
        //释放算法资源
        delete(Algorithm_);
    }

    /// @brief ROS主循环
    /// @param  
    /// @return 
    int ROS_Target_Recognition::Start(void)
    {
        ros::Duration(5).sleep();
        ImageRealTime_ = ros::Time::now();
        ImageTime_ = ros::Time::now();
        ros::Rate loop_rate(RosFrequency_);

        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
            
            //异常检测
            // if(errorDetection() == XarmErrorState_ERROR)
            // {
            //     ROS_INFO("摄像头设备异常");
            //     break;
            // }
            //判断控制状态
            if(xarmStatusDete() == XarmControlState_REJECT) continue;
            ROS_INFO("接收到目标检测请求");
            //获取机械臂识别物体的ID
            int ObjectID = getXarmObjectID();
            ros::Time   controlTime = getXarmControlTime();
            //获取图像点云数据
            cv::Mat     rgbImage;
            cv::Mat     depthImage;
            ImageTarget targetOut;
            ros::Time   imageTime;
            ros::Time   targetTime;
            if(getImageData(rgbImage, depthImage) == 0) continue;
            else{
                ROS_INFO("获取到图像信息");
                //目标识别
                imageTime    = ros::Time::now();
                // ROS_INFO("目标检测开始");
                targetOut    = targetRecAlgorithm(rgbImage, depthImage, ObjectID);
                // ROS_INFO("目标检测结束");
                targetTime   = ros::Time::now();
            }

            //目标识别结果ros输出
            rosTargetRecPublish(controlTime, imageTime, targetTime, targetOut);

        }
        return 1;
    }

    /// @brief 机械臂接受请求图像识别回调函数
    /// @param  
    void ROS_Target_Recognition::xarmGetTargetRecCallback(const commonType::XarmGetTargetConstPtr& commond_msg)
    {
        XarmGetTargetRec_ = *commond_msg;
    }  

    /// @brief 机械臂接受上次控制命令回调函数
    /// @param  
    void ROS_Target_Recognition::xarmControlCommandCallback(const commonType::XarmControlCommondConstPtr& commond_msg)
    {
        XarmControlCommondMsg_ = *commond_msg;
    }
    /// @brief 机械臂控制状态回调函数
    /// @param  
    void ROS_Target_Recognition::xarmControlStatusCallback(const commonType::XarmStatusConstPtr& status_msg)
    {
        XarmStatusMsg_ = *status_msg;
    }
    
    /// @brief 图像数据回调
    /// @param  
    void ROS_Target_Recognition::imageRgbRealsenseCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
    {
        ImageTime_ = ros::Time::now();
        cv_bridge::CvImagePtr cv_ptr; 
        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        RgbImg_ = cv_ptr->image;
        ROS_INFO("接收到rgb图像数据");
    }

    /// @brief 图像数据回调
    /// @param  
    void ROS_Target_Recognition::imageDepthRealsenseCallback(const sensor_msgs::ImageConstPtr& depth_msg)
    {
        cv_bridge::CvImagePtr cv_ptr; 
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        DepthImg_ = cv_ptr->image;
        ROS_INFO("接收到dep图像数据");
    }

    /// @brief 图像数据回调
    /// @param  
    void ROS_Target_Recognition::imageRealsenseCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
Image_Mutex_.lock();
        // ROS_INFO("接收到图像数据");
        ImageRealTime_ = ros::Time::now();
        cv_bridge::CvImagePtr cv_ptr; 

        cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        RgbImg_ = cv_ptr->image;
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        DepthImg_ = cv_ptr->image;
Image_Mutex_.unlock();
    }
    
    /// @brief 获取一帧图像数据
    /// @param  
    /// @return 
    int ROS_Target_Recognition::getImageData(cv::Mat &rgbImage, cv::Mat &depthImage)
    {
        if(!RgbImg_.data) return 0;
Image_Mutex_.lock();
        RgbImg_.copyTo(rgbImage);
        DepthImg_.copyTo(depthImage);
        RgbImg_.data = NULL;
        DepthImg_.data = NULL;
Image_Mutex_.unlock();
        return 1;
    }
    
    /// @brief 获取机械臂状态
    /// @param  
    /// @return 
    XarmCallBackState ROS_Target_Recognition::getXarmStatus(void)
    {
        commonType::XarmStatus tempXarmStatusMsg_ = XarmStatusMsg_;
        switch (tempXarmStatusMsg_.Status)
        {
            case 0:
                return XarmCallBackState_IDLE;
            case 1:
                return XarmCallBackState_WORKING;
            case -1:
                return XarmCallBackState_ERROR;
            default:
                return XarmCallBackState_IDLE;
        }
    }

    /// @brief 判断机械臂的状态
    /// @param  
    /// @return t
    XarmControlState ROS_Target_Recognition::xarmStatusDete(void)
    {
        // ROS_INFO("判断是否进行目标检测");
        commonType::XarmGetTarget   command = XarmGetTargetRec_;
        if(XarmGetTargetRec_.flag != false){
            XarmGetTargetRec_.flag = false;
            return XarmControlState_ALLOW;
        } else {
            return XarmControlState_REJECT;
        }
        return XarmControlState_REJECT;
    }
    
    /// @brief 图像检测算法
    /// @param  
    /// @return 
    ImageTarget ROS_Target_Recognition::targetRecAlgorithm(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID)
    {
        ImageTarget out;
        // 图像检测算法入口
        out = Algorithm_->Do(rgbImage, depthImage, objectID);
        return out;
    }

    /// @brief 图像检测结果ros发送
    /// @param  
    /// @return 
    int ROS_Target_Recognition::rosTargetRecPublish(ros::Time controlTime, ros::Time imageTime, ros::Time targetTime, ImageTarget data)
    {
        commonType::TargetReco       target_msg;
        target_msg.ControlStamp      = controlTime;
        target_msg.ImageStamp        = imageTime;
        target_msg.TargetStamp       = targetTime;
        target_msg.ObjectID          = data.ObjectID;  
        target_msg.sizeX             = data.sizeX;
        target_msg.sizeY             = data.sizeY;
        target_msg.x                 = data.x;
        target_msg.y                 = data.y;
        target_msg.z                 = data.z;
        target_msg.roll              = data.roll;
        target_msg.pitch             = data.pitch;
        target_msg.yaw               = data.yaw;
        target_msg.recognition_status = data.recognition_status;
        target_msg.mistake           = data.mistake;
        PubTargetRecognition_.publish(target_msg);
        //int8 recognition_status     #识别结果，0（失败）、1（成功）、-1（超时间）
        
        if(target_msg.recognition_status == 0) ROS_INFO("目标检测失败");
        else if(target_msg.recognition_status == 1) {
            ROS_INFO("目标检测成功");
            ROS_INFO("%f, %f, %f", target_msg.x, target_msg.y, target_msg.z);
        }
        else if(target_msg.recognition_status == -1)ROS_INFO("目标检测超时");
        return 1;
    }

    /// @brief 异常检测
    /// @param  
    /// @return 
    XarmErrorState ROS_Target_Recognition::errorDetection(void)
    {
        ImageTime_ = ros::Time::now();
        float time  = (ImageTime_ - ImageRealTime_).toSec();
        if((ImageTime_ - ImageRealTime_) > ros::Duration(ImageDevice_))return XarmErrorState_ERROR;
        else return XarmErrorState_NORMAL;
    }

    /// @brief 获取目标识别物体的ID号
    /// @param  
    /// @return 
    int ROS_Target_Recognition::getXarmObjectID(void)
    {
        commonType::XarmGetTarget   command = XarmGetTargetRec_;
        return command.ObjectID;
    }
    /// @brief 获取目标识别物体对应时间
    /// @param  
    /// @return 
    ros::Time ROS_Target_Recognition::getXarmControlTime(void)
    {
        commonType::XarmGetTarget   command = XarmGetTargetRec_;
        return command.stamp;
    }

    //读取参数模板
    template<typename T>
    T ROS_Target_Recognition::getParam(const std::string& name,const T& defaultValue)
    {
        T v;
        if(ros::param::get(name,v))//get parameter by name depend on ROS.
        {
            //ROS_INFO_STREAM("Found parameter: "<<name<<",\tvalue: "<<v);
            return v;
        }
        else 
            //ROS_WARN_STREAM("Cannot find value for parameter: "<<name<<",\tassigning default: "<<defaultValue);
        return defaultValue;//if the parameter haven't been set,it's value will return defaultValue.
    }

    /// @brief 记载参数
    /// @param  
    /// @return 
    Param ROS_Target_Recognition::LoadPararms(void)
    {
        Param out;
        // 参数加载
        out.Rgb_Image_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Rgb_Image_Topic",
                                        "/camera/color/image_raw");
        out.Depth_Image_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Depth_Image_Topic",
                                        "/camera/aligned_depth_to_color/image_raw");
        out.Xarm_Control_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Xarm_Control_Topic",
                                        "/xarm/XarmControl");
        out.Xarm_Status_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Xarm_Status_Topic",
                                        "/xarm/XarmControlStatus");
        out.Xarm_Recognition_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Xarm_Recognition_Topic",
                                        "/xarm/XarmTargetRec");

        out.Xarm_GetRecognition_Topic = getParam<std::string>("/target_recognition_node/Rostopic/Xarm_GetRecognition_Topic",
                                        "xarm/GetTargetRec");
        
        out.ROS_Frequency = getParam<int>("/target_recognition_node/Rosconfig/ROS_Frequency",100);
        out.Rgb_Depth_Delt_Time = getParam<int>("/target_recognition_node/Rosconfig/Rgb_Depth_Delt_Time",10);

        out.Algorithm_ID = getParam<int>("/target_recognition_node/Algorithm/Algorithm_ID",00);
        out.ImageDevice = getParam<int>("/target_recognition_node/ErrorDet/ImageDevice",3);

        out.Python_Module = getParam<std::string>("/target_recognition_node/Algorithm/Python_Module",
                                        "algorithm");
        out.Python_Class_Name = getParam<std::string>("/target_recognition_node/Algorithm/Python_Class_Name",
                                        "Algorithm");

        return out;
    }

    /// @brief 加载算法
    /// @param ID 
    /// @return 
    Image_Algorithm* ROS_Target_Recognition::loadAlgorithm(Param param)
    {
        Image_Algorithm *tempAlgorithm_ ;
    
        switch(param.Algorithm_ID)
        {
            case 00: {
                tempAlgorithm_ = new Image_AlgorithmCPP0();
                break;
            }
            case 01: {
                tempAlgorithm_ = new Image_AlgorithmPY0();
                tempAlgorithm_->LoadPython(ALGORITHM_PY0_PATH, param.Python_Module, param.Python_Class_Name);
                break;
            }
            case 10: {
                tempAlgorithm_ = new Image_AlgorithmCPP1();
                break;
            }
            case 11: {
                tempAlgorithm_ = new Image_AlgorithmPY1();
                tempAlgorithm_->LoadPython(ALGORITHM_PY1_PATH, param.Python_Module, param.Python_Class_Name);
                break;
            }
            case 20: {
                tempAlgorithm_ = new Image_AlgorithmCPP2();
                break;
            }
            case 21: {
                tempAlgorithm_ = new Image_AlgorithmPY2();
                tempAlgorithm_->LoadPython(ALGORITHM_PY2_PATH, param.Python_Module, param.Python_Class_Name);
                break;
            }
            case 30: {
                tempAlgorithm_ = new Image_AlgorithmCPP3();
                break;
            }
            case 31: {
                tempAlgorithm_ = new Image_AlgorithmPY3();
                tempAlgorithm_->LoadPython(ALGORITHM_PY3_PATH, param.Python_Module, param.Python_Class_Name);
                break;
            }
            default: {
                tempAlgorithm_ = new Image_AlgorithmCPP0();
                break;
            }
        }

        return tempAlgorithm_;
    }

}