#ifndef TARGET_RECOGNITION_ALGORITHMCPP1_H_
#define TARGET_RECOGNITION_ALGORITHMCPP1_H_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include "common.h"
#include "target_recognition_algorithm.h"
#include "darknet_alg.h" 
#include <ros/ros.h>

namespace Target_Recognition
{
    class Image_AlgorithmCPP1 : public Image_Algorithm
    {
        public:
            /// @brief 加载python路径,以及模块的名字
            /// @param path 
            void LoadPython(std::string path, std::string module_name, std::string class_name);
            
            /// @brief 获取算法名称
            /// @return 
            std::string Name();

            /// @brief 算法初始化
            /// @return 
            int Init();

            /// @brief 执行算法
            /// @return 
            ImageTarget Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID);

            /// @brief 
            /// @return 返回错误
            int ErrorDetect();

        public:
            ~Image_AlgorithmCPP1();

        private:
            /// @brief 将darkNet计算出来的值转为 算法检测需要的数据类型
            /// @param  algData darkNet计算出来的值
            /// @return 算法检测需要的数据类型
            ImageTarget getAlgResult(std::vector<detection_with_class> algData, int objectID);

        private:
            /// @brief 
            network *Net_;
            /// @brief 
            metadata Meta_;
            /// @brief 设置检测阈值
            float Thresh_        = 0.5;
            /// @brief 设置分层阈值
            float Hier_thresh_   = 0.5;
            /// @brief 设置NMS阈值
            float Nms_           = 0.45;

    };
}

#endif