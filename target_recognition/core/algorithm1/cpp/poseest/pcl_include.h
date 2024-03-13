#ifndef PCL_INCLUDE_H
#define PCL_INCLUDE_H

#include <math.h>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <vtkPlaneSource.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>      // 随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>       // 模型定义有文件
#include <pcl/sample_consensus/ransac.h>            // 采样一致性
#include <pcl/sample_consensus/sac_model_line.h>    // 拟合直线
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/segmentation/sac_segmentation.h>      // 基于采样一致性分割的类的头文件
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common_headers.h>


/*Eigen模块*/
#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace Eigen;



#endif
