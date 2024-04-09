#ifndef ARM_POSE_ESTIMATION_FINAL_H
#define ARM_POSE_ESTIMATION_FINAL_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>      // 随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>       // 模型定义有文件
#include <pcl/segmentation/sac_segmentation.h>      // 基于采样一致性分割的类的头文件
#include <pcl/common/common_headers.h>  // 包含 PCL 常见头文件
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>            // 由索引提取点云
#include <pcl/sample_consensus/ransac.h>            // 采样一致性

#include <boost/thread/thread.hpp>
#include <pcl/features/boundary.h>
#include <math.h>
#include <cmath>
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>

// #include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/ply_io.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/sample_consensus/sac_model_line.h>    // 拟合直线
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/ModelCoefficients.h>		             // 模型系数定义头文件
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/shot.h>
#include <pcl/registration/ndt.h>

/*Eigen模块*/
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "opencv2/opencv.hpp" // 包含 OpenCV 头文件

#include <fstream>  // 用于文件读取

#include <iostream>
#include <limits>
#include <vector>

// using namespace Eigen;
// using namespace cv;
// using namespace std;

void pose_est(cv::Mat &depthImage, const double x1, const double y1, const double x2, const double y2, const std::string &objectLabel, Eigen::Matrix3f &rotation_matrix, Eigen::Vector3f &translation);

Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f &R);


#endif
