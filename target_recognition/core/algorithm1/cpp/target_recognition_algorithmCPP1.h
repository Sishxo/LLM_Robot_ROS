#ifndef TARGET_RECOGNITION_ALGORITHMCPP1_H_
#define TARGET_RECOGNITION_ALGORITHMCPP1_H_

#include <string>
#include <cv_bridge/cv_bridge.h>
#include "common.h"
#include "target_recognition_algorithm.h"

namespace Target_Recognition
{
    class Image_AlgorithmCPP1 : public Image_Algorithm
    {
        public:
            /// @brief 加载python路径,以及模块的名字,以及类名
            /// @param path 
            void LoadPython(std::string path, std::string module_name, std::string class_name);
            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            std::string Name();

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            int Init();

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            ImageTarget Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID);

            /// @brief 获取当前硬件平台的名称
            /// @return 返回当前硬件平台的名称
            int ErrorDetect();
    };
}

#endif