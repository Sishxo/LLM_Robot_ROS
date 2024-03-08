#include "ConfirmPoint.h"
#include "ConfirmYaml.h"


namespace XARMGRIP_CONTROLLER
{
    /// @brief 初始化
    void ConfirmPoint::Init()
    {
        
    }
    /// @brief 关闭
    void ConfirmPoint::Close()
    {
        
    }

    /// @brief 获取yaml中点的值
    /// @param label 
    /// @param Points 
    void ConfirmPoint::GetYamlPoints(std::string label, std::vector<commonType::TargetReco> &Points)
    {
        config_ = YAML::LoadFile(CONFIRM_POINTS);
        Points.clear();
        int A_size = config_[label]["size"].as<int>() ;
        // std::cout<<"the size is "<<A_size<<std::endl;
        Points.resize(A_size);
        for(int i = 0 ; i < Points.size(); i++)
        {
            std::string point = "point" + std::to_string(i);
            Points[i].x = config_[label][point]["x"].as<float>() ;
            Points[i].y = config_[label][point]["y"].as<float>() ;
            Points[i].z = config_[label][point]["z"].as<float>() ;
            Points[i].roll = config_[label][point]["roll"].as<float>() ;
            Points[i].pitch = config_[label][point]["pitch"].as<float>() ;
            Points[i].yaw = config_[label][point]["yaw"].as<float>() ;
        }
    }
    /// @brief 更新yaml中点的值
    /// @param label 
    /// @param id 
    /// @param points 
    void ConfirmPoint::SetYamlPoint(std::string label, int id, commonType::TargetReco points)
    {
        config_ = YAML::LoadFile(CONFIRM_POINTS);
        config_[label]["size"] = std::to_string(id+1);
        config_[label]["point" + std::to_string(id)]["x"]       = std::to_string(points.x);
        config_[label]["point" + std::to_string(id)]["y"]       = std::to_string(points.y);
        config_[label]["point" + std::to_string(id)]["z"]       = std::to_string(points.z);
        config_[label]["point" + std::to_string(id)]["roll"]    = std::to_string(points.roll);
        config_[label]["point" + std::to_string(id)]["pitch"]   = std::to_string(points.pitch);
        config_[label]["point" + std::to_string(id)]["yaw"]     = std::to_string(points.yaw);
        std::ofstream fout(CONFIRM_POINTS);
        fout << config_;
    }

    /// @brief 获取label个数
    /// @param label 
    /// @param size 
    int ConfirmPoint::GetYamlPointSize(std::string label)
    {
        config_ = YAML::LoadFile(CONFIRM_POINTS);
        return config_[label]["size"].as<int>() ;
    }


    void ConfirmPoint::test()
    {

        // commonType::TargetReco temppoints;
        // temppoints.x = 1.56;
        // temppoints.y = 2.56;
        // temppoints.z = 4.56;

        // SetYamlPoint("B", 2, temppoints);


        // std::vector<commonType::TargetReco> Points;
        // GetYamlPoints("A", Points);
        // for(int i = 0 ; i< Points.size(); i++)
        // {
        //     // std::cout<<"A:"<<i<<"  "<<Points[i].x<<"  "<<Points[i].y<<"  "<<Points[i].z<<std::endl;
        // }

        // GetYamlPoints("B", Points);
        // for(int i = 0 ; i< Points.size(); i++)
        // {
        //     // std::cout<<"B:"<<i<<"  "<<Points[i].x<<"  "<<Points[i].y<<"  "<<Points[i].z<<std::endl;
        // }
    }

    /// @brief 清除所有点
    void ConfirmPoint::ClearYamlPoint(std::string label)
    {
        config_ = YAML::LoadFile(CONFIRM_POINTS);
        config_[label]["size"] = std::to_string(0);
        std::ofstream fout(CONFIRM_POINTS);
        fout << config_;
    }

}