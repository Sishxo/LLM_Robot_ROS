#include "target_recognition_algorithmPY2.h"

namespace Target_Recognition
{

    /// @brief 加载python路径,以及模块的名字
    /// @param path
    void Image_AlgorithmPY2::LoadPython(std::string path, std::string module_name, std::string class_name)
    {
        pybind11::module sys = pybind11::module::import("sys");
        sys.attr("path").attr("append")(path);
        Algorithm = pybind11::module::import(module_name.c_str()).attr(class_name.c_str());
        Algorithm_Init = Algorithm();
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    std::string Image_AlgorithmPY2::Name()
    {
        std::string name;
        pybind11::object result = Algorithm_Init.attr("Name")();
        name = result.cast<std::string>();
        return name;
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    int Image_AlgorithmPY2::Init()
    {
        pybind11::object result = Algorithm_Init.attr("Init")();
        return result.cast<int>();
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    ImageTarget Image_AlgorithmPY2::Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID)
    {
        int rgbWidth = rgbImage.size().width;
        int rgbHeight = rgbImage.size().height;
        int rgbChannels = rgbImage.channels();
        pybind11::array rgbImageData = mat_to_array(rgbImage);

        Algorithm_Init.attr("Do")(rgbImageData, rgbWidth, rgbHeight, rgbChannels, objectID);

        DetectionResult det_res;
        det_res = getDetectionResult();

        Eigen::Matrix3f rotation_matrix;
        Eigen::Vector3f translation;

        pose_est(depthImage, det_res.x1, det_res.y1, det_res.x2, det_res.y2, det_res.ObjectName, rotation_matrix, translation);

        return getData(rotation_matrix, translation, objectID, det_res.ObjectName);
    }

    /// @brief 获取当前硬件平台的名称
    /// @return 返回当前硬件平台的名称
    int Image_AlgorithmPY2::ErrorDetect()
    {
        pybind11::object result = Algorithm_Init.attr("ErrorDetect")();
        return result.cast<int>();
    }

    pybind11::array Image_AlgorithmPY2::mat_to_array(cv::Mat &mat)
    {
        int rows = mat.rows, cols = mat.cols, type = mat.type();
        pybind11::array_t<unsigned char> result({rows, cols, mat.channels()});
        auto buf = result.request();
        unsigned char *ptr = (unsigned char *)buf.ptr;
        cv::Mat mat_copy(rows, cols, type, ptr);
        mat.copyTo(mat_copy);
        return result;
    }

    DetectionResult Image_AlgorithmPY2::getDetectionResult()
    {
        pybind11::object result;
        DetectionResult out;

        result = Algorithm_Init.attr("GetX1")();
        out.x1 = result.cast<float>();
        result = Algorithm_Init.attr("GetX2")();
        out.x2 = result.cast<float>();
        result = Algorithm_Init.attr("GetY1")();
        out.y1 = result.cast<float>();
        result = Algorithm_Init.attr("GetY2")();
        out.y2 = result.cast<float>();
        result = Algorithm_Init.attr("GetDetection_status")();
        out.detection_status = result.cast<int>();
        result = Algorithm_Init.attr("GetMistake")();
        out.mistake = result.cast<float>();
        result = Algorithm_Init.attr("GetObjectID")();
        out.ObjectID = result.cast<float>();
        result = Algorithm_Init.attr("GetObjectName")();
        out.ObjectName = result.cast<std::string>();

        return out;
    }

    ImageTarget Image_AlgorithmPY2::getData(Eigen::Matrix3f &rotation_matrix, Eigen::Vector3f &translation, int objectID, std::string objectName)
    {
        Eigen::Vector3f euler;

        ImageTarget out;

        euler = rotationMatrixToEulerAngles(rotation_matrix); // return yaw-pitch-roll euler angles

        out.yaw = euler[0];
        out.pitch = euler[1];
        out.roll = euler[2];

        out.x = translation[0];
        out.y = translation[1];
        out.z = translation[2];

        out.ObjectID = objectID;


        pybind11::object result;

        result = Algorithm_Init.attr("GetRecognition_status")();
        out.recognition_status = result.cast<int>();
        result = Algorithm_Init.attr("GetMistake")();
        out.mistake = result.cast<float>();

        result = Algorithm_Init.attr("GetSizeX")();
        out.sizeX = result.cast<float>();
        result = Algorithm_Init.attr("GetSizeY")();
        out.sizeY = result.cast<float>();
        result = Algorithm_Init.attr("GetSizeZ")();
        out.sizeZ = result.cast<float>();
  
        return out;
    }

}