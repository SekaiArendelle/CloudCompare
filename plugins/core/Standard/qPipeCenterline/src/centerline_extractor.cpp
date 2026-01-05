#include "centerline_extractor.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

CenterlineExtractor::CenterlineExtractor() {
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

CenterlineExtractor::~CenterlineExtractor() {}

bool CenterlineExtractor::loadPointCloud(const std::string& filepath) {
    std::cout << "[IO] Loading point cloud: " << filepath << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filepath, *cloud_) == -1) {
        std::cerr << "[Error] Failed to load file." << std::endl;
        return false;
    }
    std::cout << "[IO] Loaded " << cloud_->size() << " points." << std::endl;
    return true;
}

void CenterlineExtractor::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
    if (!cloud) {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        return;
    }
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
}

// 核心逻辑：旋转 -> 切片 -> 计算中心 -> 逆旋转
void CenterlineExtractor::extract(float slice_resolution) {
    if (cloud_->empty()) return;
    centerline_points_.clear();

    std::cout << "[Algo] 1. Computing PCA to align tunnel..." << std::endl;
    
    // 1. 计算质心和主方向 (PCA)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_, centroid);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud_);
    Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); // 主方向矩阵

    // 2. 构建变换矩阵：将点云的主方向旋转到全局 X 轴
    // 这样我们就可以简单地沿着 X 轴进行切片，而不用处理复杂的空间几何
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -centroid[0], -centroid[1], -centroid[2]; // 先移到原点
    transform.linear() = eigen_vectors.transpose(); // 旋转对齐

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_, *transformed_cloud, transform);

    // 3. 获取旋转后点云的范围 (Min/Max X)
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);
    
    std::cout << "[Algo] 2. Slicing along aligned X-axis. Range: " 
              << min_pt.x << " to " << max_pt.x << std::endl;

    // 4. 沿 X 轴切片
    float current_x = min_pt.x;
    // 逆变换矩阵 (用于把算出来的中心点转回世界坐标)
    Eigen::Affine3f inverse_transform = transform.inverse();

    while (current_x < max_pt.x) {
        // 定义切片范围 [current_x, current_x + resolution]
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(current_x, current_x + slice_resolution);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*slice);

        // 如果切片里点太少，说明是空隙，跳过
        if (slice->size() > 10) {
            // 5. 计算该切片的中心 (在 YZ 平面上)
            Eigen::Vector3f local_center = computeSliceCenter(slice);
            // x 坐标设为切片的中点
            local_center.x() = current_x + slice_resolution / 2.0f;

            // 6. 转回世界坐标
            Eigen::Vector3f world_center = inverse_transform * local_center;
            centerline_points_.push_back(world_center);
        }

        current_x += slice_resolution;
    }

    std::cout << "[Algo] 3. Extraction complete. Generated " << centerline_points_.size() << " nodes." << std::endl;
}

// 计算切片的几何中心
// 这里实现了你说的 "最小外接圆/矩形中心" 的逻辑
// 对于对齐后的点云，就是 Y 和 Z 的 (Min+Max)/2
Eigen::Vector3f CenterlineExtractor::computeSliceCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud) {
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*slice_cloud, min_p, max_p);

    // 我们只需要 Y 和 Z 的中心，X 在外部循环控制
    float center_y = (min_p.y + max_p.y) / 2.0f;
    float center_z = (min_p.z + max_p.z) / 2.0f;

    // 注意：这里返回的 X 是 0，外部会覆盖它
    return Eigen::Vector3f(0, center_y, center_z);
}

void CenterlineExtractor::saveCenterline(const std::string& filepath) {
    std::ofstream out(filepath);
    if (!out.is_open()) return;

    // 保存为简单的 PLY 格式，只包含顶点
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << centerline_points_.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "property uchar red\n";
    out << "property uchar green\n";
    out << "property uchar blue\n";
    out << "end_header\n";

    for (const auto& p : centerline_points_) {
        // 红色中心线
        out << p.x() << " " << p.y() << " " << p.z() << " 255 0 0\n";
    }
    out.close();
    std::cout << "[IO] Centerline saved to " << filepath << std::endl;
}
