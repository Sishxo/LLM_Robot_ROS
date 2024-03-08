#ifndef TARGET_RECOGNITION_ALGORITHM_H_
#define TARGET_RECOGNITION_ALGORITHM_H_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include "common.h"

namespace Target_Recognition
{
    class Image_Algorithm
    {
        public:
            /// @brief 加载python路径,以及模块的名字
            /// @param path 
            virtual void LoadPython(std::string path, std::string module_name, std::string class_name) = 0;
            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            virtual std::string Name()  = 0;

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            virtual int Init()  = 0;

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            virtual ImageTarget Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID) = 0;

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            virtual int ErrorDetect()  = 0;
    };
}

#endif