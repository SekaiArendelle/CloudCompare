#ifndef CENTERLINE_EXTRACTOR_H
#define CENTERLINE_EXTRACTOR_H

#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

class CenterlineExtractor {
public:
    CenterlineExtractor();
    ~CenterlineExtractor();

    // 加载点云
    bool loadPointCloud(const std::string& filepath);

    // 直接设置点云（用于 CloudCompare 选择的点云）
    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

    // 核心功能：提取中心线
    // slice_resolution: 切片间隔，例如 0.5米 切一刀
    void extract(float slice_resolution);

    // 保存中心线到 PLY 文件用于查看
    void saveCenterline(const std::string& filepath);

    // 读取提取结果
    const std::vector<Eigen::Vector3f>& getCenterlinePoints() const
    {
        return centerline_points_;
    }

private:
    // 原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    
    // 提取出的中心线点集
    std::vector<Eigen::Vector3f> centerline_points_;

    // 内部辅助：计算一个 2D 切片的几何中心 (外接圆/矩形中心)
    Eigen::Vector3f computeSliceCenter(pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud);
};

#endif // CENTERLINE_EXTRACTOR_H
