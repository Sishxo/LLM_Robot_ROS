#include "target_recognition_algorithmCPP1.h"
#include "darknet_alg.h" 
#include "cfg.h"
#include <opencv2/opencv.hpp>


namespace Target_Recognition
{
    /// @brief 加载python路径,以及模块的名字
    /// @param path 
    void Image_AlgorithmCPP1::LoadPython(std::string path, std::string module_name, std::string class_name)
    {

    }

    /// @brief 获取算法名称
    /// @return 
    std::string Image_AlgorithmCPP1::Name()
    {
        return "darknet-yolo";
    }

    /// @brief 算法初始化
    /// @return 
    int Image_AlgorithmCPP1::Init()
    {
        std::string path            = ALGORITHM_PY0_PATH;
        std::string cfg_str         = path + "cfg/yolov3.cfg";
        std::string weights_str     = path + "cfg/yolov3.weights";
        std::string data_str        = path + "cfg/coco.data";

        char *cfg = const_cast<char*>(cfg_str.c_str());
        char *weights = const_cast<char*>(weights_str.c_str());
        char *data = const_cast<char*>(data_str.c_str());
        
        Net_ = load_network(cfg, weights, 0);        
        Meta_ = get_metadata(data);

        //Net_、Thresh_、Hier_thresh_打印
        ROS_INFO("网络层数: %d", Net_->n);
        ROS_INFO("学习率: %f", Net_->learning_rate);
        ROS_INFO("输入尺寸: %d", Net_->inputs);

        return 1; 
    }

    /// @brief 执行算法
    /// @return 
    ImageTarget Image_AlgorithmCPP1::Do(cv::Mat &rgbImage, cv::Mat &depthImage, int objectID)
    {
        //输出
        ImageTarget out;

        // 将cv::Mat转换为Darknet的image格式
        image img = getCvImage(rgbImage);
        // image img = load_image_color(rgbImage_path, 0, 0);

        // std::string imagePath = "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm1/test_image.jpg";
        // cv::Mat rgbImage2 = cv::imread(imagePath, cv::IMREAD_COLOR);
        // image img = getCvImage(rgbImage2);

        /**
         * 测试输入
        */
        //图像：float图像，把图像保存下来
        // cv::Mat cv_img = imageToMat(img); 
        // cv::imwrite("/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm1/saved_image.jpg", cv_img);

        std::vector<detection_with_class> algOut = detect(img, Net_, Thresh_, Hier_thresh_, Nms_, Meta_.classes);

        /**
         * 测试输出
        */
        // 打印检测结果
        std::cout<<"/******************************************/"<<std::endl;
        std::cout<<"/******************************************/"<<std::endl;

        // 打印检测结果
        // for (const auto& det : algOut) {
        //     box b = det.det.bbox;
        //     cout << "Detected: " << meta.names[det.best_class] << " ("
        //         << b.x << ", " << b.y << ", " << b.w << ", " << b.h << ")" << endl;
        // }

        // ROS_INFO("接受到的图像大小: [%f]", img.data.size());
        ROS_INFO("检测到的目标个数: [%lu]", algOut.size());

        std::cout<<"/******************************************/"<<std::endl;
        std::cout<<"/******************************************/"<<std::endl;

        out = getAlgResult(algOut, objectID);
 
        // 清理内存
        free(img.data);
        return out;
    }

    /// @brief 
    /// @return 返回错误
    int Image_AlgorithmCPP1::ErrorDetect()
    {
        return 1;
    }

    ImageTarget Image_AlgorithmCPP1::getAlgResult(std::vector<detection_with_class> algData, int objectID)
    {
        ImageTarget out;

        // 假设ImageTarget包含一个或多个目标的列表，每个目标包含位置、类别和置信度
        for (const auto& detection : algData) {

            // // 假设detection有bbox，class_id和prob属性
            // out.bbox = detection.bbox;       // 复制边界框
            // out.classes = detection.best_class; // 复制类别
            // out.confidence = detection.prob; // 复制置信度

            // 为其他字段指定固定值
            out.x = 0.0;               
            out.y = 0.0;               
            out.z = 0.0;               
            out.roll = 0.0;            
            out.pitch = 0.0;           
            out.yaw = 0.0;
            out.sizeX = 0.0;           
            out.sizeY = 0.0;             
            out.recognition_status = 1;  // 假设1表示检测成功
            out.mistake = 0.0;           
            out.ObjectID = detection.best_class; 
        }

        return out;
    }

    Image_AlgorithmCPP1::~Image_AlgorithmCPP1()
    {
        free_network(Net_);
    }

}