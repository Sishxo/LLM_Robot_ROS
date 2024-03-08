#ifndef TARGET_RECOGNITION_H_
#define TARGET_RECOGNITION_H_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "common.h"
#include "commonType/XarmControlCommond.h"
#include "commonType/XarmStatus.h"
#include "commonType/TargetReco.h"
#include "commonType/XarmGetTarget.h"
#include "target_recognition_algorithmCPP0.h"
#include "target_recognition_algorithmCPP1.h"
#include "target_recognition_algorithmCPP2.h"
#include "target_recognition_algorithmCPP3.h"

#include "target_recognition_algorithmPY0.h"
#include "target_recognition_algorithmPY1.h"
#include "target_recognition_algorithmPY2.h"
#include "target_recognition_algorithmPY3.h"
#include "py.h"
#include <mutex>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> slamSyncPolicy;

namespace Target_Recognition
{
    class ROS_Target_Recognition
    {
        public:
            /// @brief 构造函数
            ROS_Target_Recognition();
            /// @brief 析构函数
            ~ROS_Target_Recognition();

        public:
            /// @brief ROS主循环
            /// @param  
            /// @return 
            int Start(void);

        private:
            /// @brief 机械臂接受上次控制命令回调函数
            /// @param  
            void xarmControlCommandCallback(const commonType::XarmControlCommondConstPtr& commond_msg);
            /// @brief 机械臂控制状态回调函数
            /// @param  
            void xarmControlStatusCallback(const commonType::XarmStatusConstPtr& status_msg);
            /// @brief 图像数据回调
            /// @param  
            void imageRgbRealsenseCallback(const sensor_msgs::ImageConstPtr& rgb_msg);
            /// @brief 图像数据回调
            /// @param  
            void imageDepthRealsenseCallback(const sensor_msgs::ImageConstPtr& depth_msg);

            /// @brief 图像数据回调
            /// @param  
            void imageRealsenseCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg);
            /// @brief 获取一帧图像数据
            /// @param  
            /// @return 
            int getImageData(cv::Mat &rgbImage, cv::Mat &depthImage);
            /// @brief 获取一帧图像数据
            /// @param  
            /// @return 
            XarmCallBackState getXarmStatus(void);
            /// @brief 判断机械臂的状态
            /// @param  
            /// @return 
            XarmControlState xarmStatusDete(void);
            /// @brief 图像检测算法
            /// @param  
            /// @return 
            ImageTarget targetRecAlgorithm(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID);
            /// @brief 图像检测结果ros发送
            /// @param  
            /// @return 
            int rosTargetRecPublish(ros::Time controlTime, ros::Time imageTime, ros::Time targetTime, ImageTarget data);
            /// @brief 异常检测
            /// @param  
            /// @return 
            XarmErrorState errorDetection(void);

            /// @brief 获取目标识别物体的ID号
            /// @param  
            /// @return 
            int getXarmObjectID(void);

            /// @brief 读取参数模板
            /// @tparam T 
            /// @param name 
            /// @param defaultValue 
            /// @return 
            template<typename T> T getParam(const std::string& name,const T& defaultValue);

            /// @brief 记载参数
            /// @param  
            /// @return 
            Param LoadPararms(void);

            /// @brief 加载算法
            /// @param ID 
            /// @return 
            Image_Algorithm* loadAlgorithm(Param param);

            /// @brief 机械臂接受请求图像识别回调函数
            /// @param  
            void xarmGetTargetRecCallback(const commonType::XarmGetTargetConstPtr& commond_msg);
  
            /// @brief 获取目标识别物体对应时间
            /// @param  
            /// @return 
            ros::Time getXarmControlTime(void);
        private:
            /// ros相关变量
            ros::NodeHandle Node_;
            ros::Subscriber SubXarmControl_;
            ros::Subscriber SubXarmControlStatus_;
            ros::Subscriber SubXarmGetTargetRec_;
            ros::Subscriber SubImageRgbRealsense_;
            ros::Subscriber SubImageDepthRealsense_;
            ros::Publisher  PubTargetRecognition_;

            message_filters::Subscriber<sensor_msgs::Image>* rgb_sub_ ;             // topic1 输入
            message_filters::Subscriber<sensor_msgs::Image>* depth_sub_;               // topic2 输入
            message_filters::Synchronizer<slamSyncPolicy>* sync_;

            int RosFrequency_;
            commonType::XarmGetTarget        XarmGetTargetRec_;
            commonType::XarmControlCommond   XarmControlCommondMsg_;
            commonType::XarmStatus           XarmStatusMsg_;
            cv::Mat RgbImg_;
            cv::Mat DepthImg_;

            ros::Time   ImageTime_;
            ros::Time   ImageRealTime_;
            float ImageDevice_;

            std::mutex Image_Mutex_;



        private:
            /// @brief 算法 
            Image_Algorithm *Algorithm_;
    };
}

#endif