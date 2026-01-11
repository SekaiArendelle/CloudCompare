#pragma once
#include <ccPointCloud.h>
#include <QString>
#include <vector>

// PCL Headers needed for the class members if any
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class CrossSectionExtractor
{
public:
    // 对应 Dialog 的参数
    struct Parameters
    {
        double fittingThreshold = 0.05;
        double denoiseLevel = 1.0;
    };

    explicit CrossSectionExtractor( const Parameters& params );
    ~CrossSectionExtractor() = default;

    // 统一入口：输入CC点云，执行所有智能操作
    void processTunnel( ccPointCloud* inputCloud, ccPointCloud*& outputCloud, QString& reportText );

    QString getLastError() const { return m_lastError; }

private:
    Parameters m_params;
    QString m_lastError;

    // 内部类型定义
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;
    enum TunnelType { TYPE_UNKNOWN=0, TYPE_RECTANGLE, TYPE_ARCH, TYPE_CIRCLE };

    // 内部算法函数 (移植自你的 TunnelProcessor)
    void adaptiveProcess(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out);
    void alignPerfectly(PointCloudT::Ptr cloud);
    TunnelType detectType(PointCloudT::Ptr cloud);
    std::string getShapeName(TunnelType type);
};
