#include "target_recognition_algorithmCPP1.h"


namespace Target_Recognition
{
    /// @brief 加载python路径,以及模块的名字
    /// @param path 
    void Image_AlgorithmCPP1::LoadPython(std::string path, std::string module_name, std::string class_name)
    {
        
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    std::string Image_AlgorithmCPP1::Name()
    {
        return "算法1-cpp";
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    int Image_AlgorithmCPP1::Init()
    {
        return 1;
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    ImageTarget Image_AlgorithmCPP1::Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID)
    {
        ImageTarget out ;

        return out;
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    int Image_AlgorithmCPP1::ErrorDetect()
    {
        return 1;
    }
}