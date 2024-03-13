#include <iostream>
#include <chrono>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "pcl_include.h"  // 包含 PCL 头文件
#include "opencv2/opencv.hpp"  // 包含 OpenCV 头文件

using namespace cv;
using namespace std;

//yaml文件中的参数设置
struct Parameters {
    struct CameraParameters {
        double camera_cx;
        double camera_cy;
        double camera_fx;
        double camera_fy;
    } camera_parameters;

    struct FilteringParameters {
        struct ModelFilter {
            double max_z;
        } model_filter;

        struct TargetFilter {
            double max_z;
            double max_y;
            double min_x;
            double max_x;            
        } target_filter;

        struct VoxelGrid {
            std::vector<double> leaf_size_model;
            std::vector<double> leaf_size_target;
        } voxel_grid;
    }filtering_parameters;

    struct ClusterParameters {
        double cluster_tolerance;
        int min_cluster_size;
        int max_cluster_size;
    } cluster_parameters;

    struct GICPParameters {
        int max_iterations;
    } gicp_parameters;

    struct ModelParameters {
        struct Model {
            double x_min;
            double x_max;
            double y_min;
            double y_max;
            double z_min;
            double z_max;
        } model;
    } model_parameters;
};

namespace YAML {
    template <>
    struct convert<Parameters> {
        static Node encode(const Parameters& rhs) {
            Node node;
            node["camera_parameters"]["camera_cx"] = rhs.camera_parameters.camera_cx;
            node["camera_parameters"]["camera_cy"] = rhs.camera_parameters.camera_cy;
            node["camera_parameters"]["camera_fx"] = rhs.camera_parameters.camera_fx;
            node["camera_parameters"]["camera_fy"] = rhs.camera_parameters.camera_fy;
            node["filtering_parameters"]["model_filter"]["max_z"] = rhs.filtering_parameters.model_filter.max_z;
            node["filtering_parameters"]["target_filter"]["max_z"] = rhs.filtering_parameters.target_filter.max_z;
            node["filtering_parameters"]["target_filter"]["max_y"] = rhs.filtering_parameters.target_filter.max_y;
            node["filtering_parameters"]["target_filter"]["min_x"] = rhs.filtering_parameters.target_filter.min_x;
            node["filtering_parameters"]["target_filter"]["max_x"] = rhs.filtering_parameters.target_filter.max_x;
            node["filtering_parameters"]["voxel_grid"]["leaf_size_model"] = rhs.filtering_parameters.voxel_grid.leaf_size_model;
            node["filtering_parameters"]["voxel_grid"]["leaf_size_target"] = rhs.filtering_parameters.voxel_grid.leaf_size_target;
            node["cluster_parameters"]["cluster_tolerance"] = rhs.cluster_parameters.cluster_tolerance;
            node["cluster_parameters"]["min_cluster_size"] = rhs.cluster_parameters.min_cluster_size;
            node["cluster_parameters"]["max_cluster_size"] = rhs.cluster_parameters.max_cluster_size;
            node["gicp_parameters"]["max_iterations"] = rhs.gicp_parameters.max_iterations;
            node["model_parameters"]["model"]["x_min"] = rhs.model_parameters.model.x_min;
            node["model_parameters"]["model"]["x_max"] = rhs.model_parameters.model.x_max;
            node["model_parameters"]["model"]["y_min"] = rhs.model_parameters.model.y_min;
            node["model_parameters"]["model"]["y_max"] = rhs.model_parameters.model.y_max;
            node["model_parameters"]["model"]["z_min"] = rhs.model_parameters.model.z_min;
            node["model_parameters"]["model"]["z_max"] = rhs.model_parameters.model.z_max;


            return node;
        }

        static bool decode(const Node& node, Parameters& rhs) {
            if (!node.IsMap() || !node["filtering_parameters"].IsDefined() || !node["gicp_parameters"].IsDefined() ) {
                return false;
            }
            rhs.camera_parameters.camera_cx = node["camera_parameters"]["camera_cx"].as<double>();
            rhs.camera_parameters.camera_cy = node["camera_parameters"]["camera_cy"].as<double>();
            rhs.camera_parameters.camera_fx = node["camera_parameters"]["camera_fx"].as<double>();
            rhs.camera_parameters.camera_fy = node["camera_parameters"]["camera_fy"].as<double>();
            rhs.filtering_parameters.model_filter.max_z = node["filtering_parameters"]["model_filter"]["max_z"].as<double>();
            rhs.filtering_parameters.target_filter.max_z = node["filtering_parameters"]["target_filter"]["max_z"].as<double>();
            rhs.filtering_parameters.target_filter.max_y = node["filtering_parameters"]["target_filter"]["max_y"].as<double>();
            rhs.filtering_parameters.target_filter.min_x = node["filtering_parameters"]["target_filter"]["min_x"].as<double>();
            rhs.filtering_parameters.target_filter.max_x = node["filtering_parameters"]["target_filter"]["max_x"].as<double>();
            rhs.filtering_parameters.voxel_grid.leaf_size_model = node["filtering_parameters"]["voxel_grid"]["leaf_size_model"].as<std::vector<double>>();
            rhs.filtering_parameters.voxel_grid.leaf_size_target = node["filtering_parameters"]["voxel_grid"]["leaf_size_target"].as<std::vector<double>>();
            rhs.cluster_parameters.cluster_tolerance = node["cluster_parameters"]["cluster_tolerance"].as<double>();
            rhs.cluster_parameters.min_cluster_size = node["cluster_parameters"]["min_cluster_size"].as<int>();
            rhs.cluster_parameters.max_cluster_size = node["cluster_parameters"]["max_cluster_size"].as<int>();
            rhs.gicp_parameters.max_iterations = node["gicp_parameters"]["max_iterations"].as<int>();
            rhs.model_parameters.model.x_min = node["model_parameters"]["model"]["x_min"].as<double>();
            rhs.model_parameters.model.x_max = node["model_parameters"]["model"]["x_max"].as<double>();
            rhs.model_parameters.model.y_min = node["model_parameters"]["model"]["y_min"].as<double>();
            rhs.model_parameters.model.y_max = node["model_parameters"]["model"]["y_max"].as<double>();
            rhs.model_parameters.model.z_min = node["model_parameters"]["model"]["z_min"].as<double>();
            rhs.model_parameters.model.z_max = node["model_parameters"]["model"]["z_max"].as<double>();
            return true;
        }
    };
}
