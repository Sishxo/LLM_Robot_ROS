#include "arm_pose_estimation_final.h"
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
// 相机内参
const double global_camera_cx = 645.882;
const double global_camera_cy = 373.954;
const double global_camera_fx = 907.996;
const double global_camera_fy = 905.985;

// 1. 深度图像转换为点云
// 输入：深度图、x1,y1,x2,y2，输出：实际物体的点云
void depthImg_to_PcdCloud(const cv::Mat depth, const double x1, const double y1, const double x2, const double y2, pcl::PointCloud<pcl::PointXYZ>::Ptr &actual_pointcloud)
{
    if (depth.empty())
    { // 检查深度图像是否为空
        printf("The img depth is empty.\n");
        return;
    }

    // 遍历矩形框内的像素，转换为点云
    for (int m = y1; m < y2; m++)
    {
        for (int n = x1; n < x2; n++)
        {
            // 获取深度图像的值
            ushort d = depth.ptr<ushort>(m)[n];

            // 如果深度值为0，跳过
            if (d == 0)
                continue;

            // 创建点云点
            pcl::PointXYZ p;

            // 计算点云点的三维坐标
            p.z = double(d);
            p.x = (n - global_camera_cx) * p.z / global_camera_fx;
            p.y = (m - global_camera_cy) * p.z / global_camera_fy;

            // 将点云点添加到点云中
            actual_pointcloud->points.push_back(p);
        }
    }
}

// 2. 对实际点云进行滤波、对实际点云聚类
// 输入：物体的点云 输出：滤波聚类之后的实际点云
void registerPointClouds(const pcl::PointCloud<pcl::PointXYZ>::Ptr &actual_pointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &processed_actual_pointcloud)
{

    // 对实际点云体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_actual_pointcloud;
    voxel_filter_actual_pointcloud.setInputCloud(actual_pointcloud);
    voxel_filter_actual_pointcloud.setLeafSize(3.0, 3.0, 3.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_actual_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_actual_pointcloud.filter(*filtered_actual_pointcloud);

    // 对实际点云进行聚类
    //  创建KD树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(filtered_actual_pointcloud);

    // 创建一个聚类对象
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(4.0);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(10000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(filtered_actual_pointcloud);
    ec.extract(cluster_indices);

    // 找到最大的聚类团
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &index : cluster_indices[i].indices)
        {
            cluster->points.push_back(filtered_actual_pointcloud->points[index]);
        }
        clusters.push_back(cluster);
    }

    // 对聚类团按大小进行排序，最大的在前面
    std::sort(clusters.begin(), clusters.end(), [](const pcl::PointCloud<pcl::PointXYZ>::Ptr &a, const pcl::PointCloud<pcl::PointXYZ>::Ptr &b)
              { return a->size() > b->size(); });
    std::cout << clusters.size() << std::endl;
    // 获取最大的聚类团
    processed_actual_pointcloud = clusters[0];
}

// 3. 使用平面拟合的方法求出平面的法向量，并得到该平面的长宽，使用拟合平面的长 宽分割出一个平面用于之后的ICP匹配；使用求出的长宽找到CAD模型对应的长宽，找到对应的CAD平面并进行滤波，先前设置的CAD模型该平面的法向量与步骤一求出的法向量求出R1
// 输入：物体名字、滤波聚类之后的实际点云 输出：旋转矩阵R1 分割出的平面 分割出的用于ICP的平面 对应CAD平面的长度和宽度）
void computeRotationMatrix_First(const std::string &objectLabel, const pcl::PointCloud<pcl::PointXYZ>::Ptr &processed_actual_pointcloud, Eigen::Matrix3f &rotation_matrix_first, pcl::PointCloud<pcl::PointXYZ>::Ptr &final_plane, pcl::PointCloud<pcl::PointXYZ>::Ptr &CAD_plane)
{
    std::vector<std::string> object_names = {"biscuit", "cake", "waffle", "lemon_tea", "milk"};
    std::vector<std::vector<std::vector<float>>> object_parts = {
        {{208, 50}, {208, 47}, {50, 47}},
        {{235, 104}, {235, 53}, {104, 53}},
        {{195, 90}, {195, 38}, {90, 38}},
        {{106, 62}, {106, 40}, {62, 40}},
        {{130, 53}, {130, 38}, {53, 38}}};
    std::vector<std::vector<std::vector<int>>> object_normals = {
        {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}},
        {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}},
        {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}},
        {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}},
        {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}}};

    std::vector<std::vector<std::string>> object_plane = {
        {"/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/biscuit1.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/biscuit2.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/biscuit3.pcd"},
        {"/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/cake1.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/cake2.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/cake3.pcd"},
        {"/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/waffle1.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/waffle2.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/waffle3.pcd"},
        {"/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/lemon_tea1.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/lemon_tea2.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/lemon_tea3.pcd"},
        {"/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/milk1.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/milk2.pcd", "/home/orin1/data/xarm_ws/src/target_recognition/core/algorithm2/py/ICP_depend/CAD_model/milk3.pcd"}};

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(2);

    // 设置输入点云数据
    seg.setInputCloud(processed_actual_pointcloud);

    // 执行平面分割
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    // 计算平面法向量
    Eigen::Vector3f plane_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // 创建分割出的平面点云

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(processed_actual_pointcloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane_cloud);

    Eigen::Vector3f up_vector(0.0, 0.0, 1.0); // Z-axis
    Eigen::Quaternionf q;
    q.setFromTwoVectors(plane_normal, up_vector);

    // Step 2: Apply the rotation to the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*plane_cloud, *rotated_cloud, Eigen::Vector3f::Zero(), q);

    // Step 3: Compute dimensions (length and width) on the XY plane
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*rotated_cloud, min_pt, max_pt);

    // Calculate length and width
    float plane_length = max_pt.x - min_pt.x;
    float plane_width = max_pt.y - min_pt.y;

    // 将较长的那个设为长度
    if (plane_width > plane_length)
    {
        std::swap(plane_length, plane_width); // Swap the values if width is longer
    }

    std::cout << "Plane Length: " << plane_length << std::endl;
    std::cout << "Plane Width: " << plane_width << std::endl;

    // 提取从min_pt.x到max_pt.x，min_pt.y到max_pt.y的平面，并将其旋转回原来的实际点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr optimized_plane(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < rotated_cloud->size(); ++i)
    {
        const pcl::PointXYZ &point = rotated_cloud->points[i];
        if (point.x >= min_pt.x && point.x <= max_pt.x && point.y >= min_pt.y && point.y <= max_pt.y)
        {
            optimized_plane->push_back(point);
        }
    }

    pcl::transformPointCloud(*optimized_plane, *final_plane, Eigen::Vector3f::Zero(), q.inverse());

    // 找到CAD模型中与该物体对应的面的对应法向量

    // 找到与objectLabel匹配的物体名称
    std::string matched_object_name;
    for (const auto &object_name : object_names)
    {
        if (object_name == objectLabel)
        {
            matched_object_name = object_name;
            break;
        }
    }

    // 先找物体
    int matched_object_index = -1;
    for (size_t i = 0; i < object_names.size(); ++i)
    {
        if (object_names[i] == matched_object_name)
        {
            matched_object_index = i;
            break;
        }
    }

    if (matched_object_index == -1)
    {
        std::cerr << "Error: No matching object found for the PCD filename." << std::endl;
    }

    // 再找面
    Eigen::Vector3f closest_normal;
    float min_length_diff = std::numeric_limits<float>::max();
    float min_width_diff = std::numeric_limits<float>::max();
    std::vector<float> closest_parts(2);
    for (const auto &parts : object_parts[matched_object_index])
    {
        float length_diff = std::abs(parts[0] - plane_length);
        float width_diff = std::abs(parts[1] - plane_width);
        if (length_diff < min_length_diff)
        {
            min_length_diff = length_diff;
            closest_parts[0] = parts[0];
        }
        if (width_diff < min_width_diff)
        {
            min_width_diff = width_diff;
            closest_parts[1] = parts[1];
        }
    }
    float closest_parts_length;
    float closest_parts_width;
    closest_parts_length = closest_parts[0];
    closest_parts_width = closest_parts[1];

    std::cout << "Closest parts length: " << closest_parts_length << std::endl;
    std::cout << "Closest parts width: " << closest_parts_width << std::endl;

    // 最后找向量
    int closest_parts_index = -1;
    for (size_t i = 0; i < object_parts[matched_object_index].size(); ++i)
    {
        if (object_parts[matched_object_index][i] == closest_parts)
        {
            closest_parts_index = i;
            break;
        }
    }

    if (closest_parts_index == -1)
    {
        std::cerr << "Error: No matching parts found for the closest dimensions." << std::endl;
    }

    closest_normal = Eigen::Vector3f(
        object_normals[matched_object_index][closest_parts_index][0],
        object_normals[matched_object_index][closest_parts_index][1],
        object_normals[matched_object_index][closest_parts_index][2]);

    // 加载找到的平面的CAD模型
    std::string cad_model_path = object_plane[matched_object_index][closest_parts_index];

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(cad_model_path, *CAD_plane) == -1)
    {
        PCL_ERROR("Couldn't read CAD model file\n");
        return; // 加载失败时的处理，可以根据实际情况进行修改
    }

    // 对这个平面滤波
    //  对CAD模型的点云进行体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_process_CAD_plane;
    voxel_process_CAD_plane.setInputCloud(CAD_plane);
    voxel_process_CAD_plane.setLeafSize(3.0, 3.0, 3.0);
    voxel_process_CAD_plane.filter(*CAD_plane);

    // 计算旋转矩阵
    Eigen::Vector3f axis = closest_normal.cross(plane_normal);
    float angle = std::acos(closest_normal.dot(plane_normal));
    Eigen::AngleAxisf rotation(angle, axis);
    rotation_matrix_first = rotation.matrix();

    // std::cout << "Rotation matrix:\n" << rotation_matrix_first << std::endl;
}

// 4. 对CAD平面进行旋转和平移
// 输入：物体名字objectLabel 上一个函数计算得到的长度和宽度 滤波完成的CAD模型processed_CAD_pointcloud 分割得到的平面plane_cloud 旋转矩阵R1 rotation_matrix_first 输出：变换之后的CAD模型transformed_CAD_pointcloud ,变换之后的CAD平面 平移向量T1 translation_first）
void transformCADModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &final_plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr &CAD_plane, const Eigen::Matrix3f &rotation_matrix_first, pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_CAD_plane, Eigen::Vector3f &translation_first)
{

    // 计算分割得到的平面的质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*final_plane, centroid);

    translation_first << centroid[0], centroid[1], centroid[2];

    // 应用旋转矩阵到点云中的每个点
    for (size_t i = 0; i < CAD_plane->points.size(); ++i)
    {
        Eigen::Vector3f transformed_CAD_plane_point(CAD_plane->points[i].x, CAD_plane->points[i].y, CAD_plane->points[i].z);
        transformed_CAD_plane_point = rotation_matrix_first * transformed_CAD_plane_point;
        CAD_plane->points[i].x = transformed_CAD_plane_point[0];
        CAD_plane->points[i].y = transformed_CAD_plane_point[1];
        CAD_plane->points[i].z = transformed_CAD_plane_point[2];
    }

    pcl::transformPointCloud(*CAD_plane, *transformed_CAD_plane, translation_first, Eigen::Quaternionf::Identity());
}

// 5. ICP匹配 （输入：旋转平移变换之后的CAD模型 滤波聚类之后的实际点云 输出：旋转矩阵R2 T2）
void ICPRegistration(pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed_CAD_plane, const pcl::PointCloud<pcl::PointXYZ>::Ptr &final_plane, Eigen::Matrix3f &rotation_matrix_second, Eigen::Vector3f &translation_second)
{
    // 创建 ICP 对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    // 设置源和目标点云
    icp.setInputSource(final_plane);
    icp.setInputTarget(transformed_CAD_plane);

    // 设置ICP参数
    // icp.setMaxCorrespondenceDistance(0.05); // 设置最大对应距离
    icp.setMaximumIterations(200); // 设置最大迭代次数
    // icp.setTransformationEpsilon(1e-8); // 设置收敛判定条件

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*cloud_aligned); // 源点云变换之后的结果

    // // 输出配准结果
    // std::cout << "Converged: " << icp.hasConverged() << std::endl;
    // std::cout << "Fitness Score: " << icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

    // 从变换矩阵中提取旋转矩阵和平移向量
    rotation_matrix_second = transformation_matrix.block<3, 3>(0, 0); // 提取旋转矩阵
    translation_second = transformation_matrix.block<3, 1>(0, 3);     // 提取平移向量

    // // 可视化
    // pcl::visualization::PCLVisualizer viewer("ICP Registration");
    // viewer.setBackgroundColor(0.0, 0.0, 0.0); // 设置背景颜色为黑色

    // // 可视化分割出的，ICP匹配之后的平面
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> actual_color_handler(cloud_aligned, 255, 255, 255);
    // viewer.addPointCloud(cloud_aligned, actual_color_handler, "cloud_aligned");

    // // 可视化CAD平面
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_CAD_plane_color_handler(transformed_CAD_plane, 0, 0, 255);
    // viewer.addPointCloud(transformed_CAD_plane, transformed_CAD_plane_color_handler, "transformed_CAD_plane");

    // // 显示
    // viewer.spin();
}

// 6. 计算最终的R和T（输入：R1，R2，T1，T2 输出：最终的R T）
// 注意R1 T1是CAD模型到实际点云（物体坐标系到相机坐标系）；R2 T2是实际点云到CAD模型（相机坐标系到物体坐标系）；记得进行变换
void computeTotalTransformation(const Eigen::Matrix3f &rotation_matrix_first, const Eigen::Vector3f &translation_first, const Eigen::Matrix3f &rotation_matrix_second, const Eigen::Vector3f &translation_second, Eigen::Matrix3f &total_rotation, Eigen::Vector3f &total_translation)
{
    // 计算第二个旋转矩阵的逆
    Eigen::Matrix3f inverse_rotation_second = rotation_matrix_second.inverse();

    // 计算总的旋转矩阵
    total_rotation = inverse_rotation_second * rotation_matrix_first;

    // 计算总的平移向量
    total_translation = inverse_rotation_second * (translation_first - translation_second);

    std::cout << "Rotation matrix:\n"
              << total_rotation << std::endl;
    std::cout << "Translation:\n"
              << total_translation << std::endl;
}

// 7. 立方体总函数（输入：深度图、物体名字、识别出的物体边界框数值x1,x2,y1,y2 输出：旋转和平移数值）
void cuboid_Function(const cv::Mat depth, const double x1, const double y1, const double x2, const double y2, const std::string &objectLabel, Eigen::Matrix3f &total_rotation, Eigen::Vector3f &total_translation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr actual_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_actual_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_CAD_plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr CAD_plane(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix3f rotation_matrix_first;
    Eigen::Matrix3f rotation_matrix_second;
    Eigen::Vector3f translation_first;
    Eigen::Vector3f translation_second;

    // Call your functions in the appropriate order
    depthImg_to_PcdCloud(depth, x1, y1, x2, y2, actual_pointcloud);
    registerPointClouds(actual_pointcloud, processed_actual_pointcloud);
    computeRotationMatrix_First(objectLabel, processed_actual_pointcloud, rotation_matrix_first, final_plane, CAD_plane);
    transformCADModel(final_plane, CAD_plane, rotation_matrix_first, transformed_CAD_plane, translation_first);
    ICPRegistration(transformed_CAD_plane, final_plane, rotation_matrix_second, translation_second);
    computeTotalTransformation(rotation_matrix_first, translation_first, rotation_matrix_second, translation_second, total_rotation, total_translation);
}

// 圆柱体总函数
void cylinder_Function(const cv::Mat depth, const double x1, const double y1, const double x2, const double y2, const std::string &objectLabel, Eigen::Vector3f &cylinder_total_translation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_actual_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cylinder_actual_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

    depthImg_to_PcdCloud(depth, x1, y1, x2, y2, cylinder_actual_pointcloud);
    registerPointClouds(cylinder_actual_pointcloud, processed_cylinder_actual_pointcloud);

    Eigen::Vector4f cylinder_centroid;
    pcl::compute3DCentroid(*processed_cylinder_actual_pointcloud, cylinder_centroid);

    cylinder_total_translation << cylinder_centroid[0], cylinder_centroid[1], cylinder_centroid[2];

    std::cout << "Cylinder_Translation:\n"
              << cylinder_total_translation << std::endl;
}

void pose_est(cv::Mat &depthImage, const double x1, const double y1, const double x2, const double y2, const std::string &objectLabel, Eigen::Matrix3f &rotation_matrix, Eigen::Vector3f &translation)
{
    if (objectLabel == "cake" || objectLabel == "milk" || objectLabel == "lemon_tea" || objectLabel == "biscuit" || objectLabel == "waffle")
    {
        cuboid_Function(depthImage, x1, y1, x2, y2, objectLabel, rotation_matrix, translation);
    }
    else if (objectLabel == "mushroom_soup" || objectLabel == "tape" || objectLabel == "coke")
    {
        cylinder_Function(depthImage, x1, y1, x2, y2, objectLabel, translation);
    }
}

Eigen::Vector3f rotationMatrixToEulerAngles(const Eigen::Matrix3f &R)
{
    Eigen::Vector3f euler;

    // 先计算 pitch
    euler[1] = asin(-R(2, 0)); // pitch

    // 计算 yaw 和 roll
    if (cos(euler[1]) > 1e-6) // 如果 pitch 不是 90 度或 -90 度
    {
        euler[0] = atan2(R(2, 1) / cos(euler[1]), R(2, 2) / cos(euler[1])); // yaw
        euler[2] = atan2(R(1, 0) / cos(euler[1]), R(0, 0) / cos(euler[1])); // roll
    }
    else // 如果 pitch 是 90 度或 -90 度
    {
        euler[0] = atan2(-R(1, 2), R(1, 1)); // yaw
        euler[2] = 0; // roll
    }

    return euler;
}