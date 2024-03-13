#include "box-ICP2-up-new.h"

// 计算点云的直径
double computePointCloudDiameter(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    double max_distance_squared = 0.0;

    for (const auto& point : cloud.points) {
        double distance_squared = point.x * point.x + point.y * point.y + point.z * point.z;
        max_distance_squared = std::max(max_distance_squared, distance_squared);
    }

    return std::sqrt(max_distance_squared);
}

// 对点云绕X轴旋转
void rotatePointCloudX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double angle_degree) {
    double angle_rad = pcl::deg2rad(angle_degree);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitX()));

    pcl::transformPointCloud(*cloud, *cloud, transform);
}

// 对点云绕Y轴旋转
void rotatePointCloudY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double angle_degree) {
    double angle_rad = pcl::deg2rad(angle_degree);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f::UnitY()));

    pcl::transformPointCloud(*cloud, *cloud, transform);
}

int main() {
    // 读取源点云和目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile<pcl::PointXYZ>("obj_CAD.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("obj.pcd", *target_cloud);

    for (auto& point : source_cloud->points) {
        point.x /= 1000;
        point.y /= 1000;
        point.z /= 1000;
    }

    // for (auto& point : target_cloud->points) {
    //     point.x *= 1000;
    //     point.y *= 1000;
    //     point.z *= 1000;
    // }

    // 对CAD模型绕Y轴旋转90度
    rotatePointCloudX(source_cloud, 90.0); // 旋转90度
    rotatePointCloudY(source_cloud, 180.0); // 旋转90度

    // 创建 ICP 对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // 设置源和目标点云
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);

    // 设置ICP参数
    // icp.setMaxCorrespondenceDistance(0.05); // 设置最大对应距离
    icp.setMaximumIterations(50); // 设置最大迭代次数
    // icp.setTransformationEpsilon(1e-8); // 设置收敛判定条件

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned);

    // 输出配准结果
    std::cout << "Converged: " << icp.hasConverged() << std::endl;
    std::cout << "Fitness Score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation Matrix:\n" << icp.getFinalTransformation() << std::endl;

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("ICP Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    viewer->addPointCloud(source_cloud, "source_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "source_cloud");

    viewer->addPointCloud(target_cloud, "target_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "target_cloud");

    viewer->addPointCloud(cloud_aligned, "cloud_aligned");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud_aligned");

    // 使用点云的直径设置坐标系大小
    double point_cloud_diameter = computePointCloudDiameter(*source_cloud);
    viewer->addCoordinateSystem(point_cloud_diameter);

    viewer->initCameraParameters();

    viewer->spin();

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0; // 表示程序成功结束
}
