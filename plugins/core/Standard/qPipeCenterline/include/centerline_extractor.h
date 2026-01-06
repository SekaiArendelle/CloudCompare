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

    // 获取所有分支的中心线
    const std::vector<std::vector<Eigen::Vector3f>>& getCenterlineTracks() const
    {
        return centerline_tracks_;
    }

    // 保存中心线到 PLY 文件用于查看
    void saveCenterline(const std::string& filepath);

    // 读取提取结果
    const std::vector<Eigen::Vector3f>& getCenterlinePoints() const
    {
        return centerline_points_;
    }

private:
    struct Track
    {
        std::vector<Eigen::Vector3f> nodes; // 世界坐标
        Eigen::Vector2f lastYZ;             // 对齐坐标系下的 YZ
        int idle = 0;                       // 连续未匹配的切片数
        bool active = true;
    };

    // 原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    
    // 提取出的中心线点集（为了兼容，保留最长主干）
    std::vector<Eigen::Vector3f> centerline_points_;

    // 所有分支中心线
    std::vector<std::vector<Eigen::Vector3f>> centerline_tracks_;

    // 内部辅助：计算一个 2D 切片的几何中心 (外接圆/矩形中心)
    // prevCenterYZ: 之前切片的中心（对齐坐标系下的 YZ），用于跟踪主干，空指针表示无跟踪
    Eigen::Vector3f computeSliceCenter(
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud,
        const Eigen::Vector2f* prevCenterYZ,
        float followRadiusSq,
        std::size_t minFollowPoints);
};

#endif // CENTERLINE_EXTRACTOR_H
