#include "../include/CrossSectionExtractor.h"

// PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>

// Qt
#include <QDir>
#include <QDateTime>

CrossSectionExtractor::CrossSectionExtractor( const Parameters& params )
    : m_params( params )
{
}

std::string CrossSectionExtractor::getShapeName(TunnelType type) {
    switch (type) {
        case TYPE_RECTANGLE: return "Rectangle (矩形)";
        case TYPE_ARCH:      return "Arch (拱形)";
        case TYPE_CIRCLE:    return "Circle (圆形)";
        default:             return "Unknown";
    }
}

void CrossSectionExtractor::adaptiveProcess(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_out) {
    int total_points = cloud_in->size();
    
    // 如果点太少，只做简单处理
    if (total_points < 3000) {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_in);
        vg.setLeafSize(0.005f, 0.005f, 0.005f);
        vg.filter(*cloud_out);
    }
    else {
        // 密集模式：MLS 修复
        PointCloudT::Ptr temp1(new PointCloudT);
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(cloud_in);
        vg.setLeafSize(0.02f, 0.02f, 0.02f);
        vg.filter(*temp1);

        PointCloudT::Ptr temp2(new PointCloudT);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(temp1);
        sor.setMeanK(50);
        // 【关键】使用界面传入的去噪参数
        sor.setStddevMulThresh(m_params.denoiseLevel); 
        sor.filter(*temp2);

        pcl::MovingLeastSquares<PointT, PointT> mls;
        mls.setInputCloud(temp2);
        mls.setComputeNormals(false);
        mls.setPolynomialOrder(2);
        mls.setSearchRadius(0.05);
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::NONE);
        mls.process(*cloud_out);
    }
}

void CrossSectionExtractor::alignPerfectly(PointCloudT::Ptr cloud) {
    if (cloud->empty()) return;
    
    pcl::PCA<PointT> pca; 
    pca.setInputCloud(cloud);
    Eigen::Matrix3f vec = pca.getEigenVectors();
    Eigen::Vector4f cen; 
    pcl::compute3DCentroid(*cloud, cen);
    
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.block<3,3>(0,0) = vec.transpose(); 
    tf.block<3,1>(0,3) = -1.0f * (tf.block<3,3>(0,0) * cen.head<3>());
    pcl::transformPointCloud(*cloud, *cloud, tf);

    // RANSAC 校正
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC); 
    // 【关键】使用界面传入的拟合阈值
    seg.setDistanceThreshold(m_params.fittingThreshold); 
    seg.setInputCloud(cloud); 
    seg.segment(*inliers, *coeff);

    if (!inliers->indices.empty()) {
        float dx = coeff->values[3], dy = coeff->values[4];
        float angle_rad = atan2(dy, dx);
        float angle_deg = angle_rad * 180.0 / M_PI;
        float target_angle = 0.0f;
        if (std::abs(angle_deg) <= 45.0) target_angle = 0.0f; 
        else if (angle_deg > 45.0) target_angle = M_PI / 2.0f; 
        else target_angle = -M_PI / 2.0f; 

        float rotation_needed = target_angle - angle_rad;
        Eigen::Affine3f rot = Eigen::Affine3f::Identity();
        rot.rotate(Eigen::AngleAxisf(rotation_needed, Eigen::Vector3f::UnitZ()));
        pcl::transformPointCloud(*cloud, *cloud, rot);
    }
    
    PointT minP, maxP; pcl::getMinMax3D(*cloud, minP, maxP);
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation() << -(minP.x+maxP.x)/2.0, -(minP.y+maxP.y)/2.0, 0.0;
    pcl::transformPointCloud(*cloud, *cloud, trans);
}

CrossSectionExtractor::TunnelType CrossSectionExtractor::detectType(PointCloudT::Ptr cloud) {
    if (cloud->empty()) return TYPE_UNKNOWN;
    PointT minPt, maxPt; pcl::getMinMax3D(*cloud, minPt, maxPt);
    float h = maxPt.y - minPt.y;
    
    pcl::PassThrough<PointT> pass; 
    pass.setInputCloud(cloud); 
    pass.setFilterFieldName("y");
    pass.setFilterLimits(maxPt.y - h*0.05, maxPt.y); 
    PointCloudT::Ptr top(new PointCloudT); pass.filter(*top);
    
    pcl::SACSegmentation<PointT> seg; 
    pcl::ModelCoefficients::Ptr c(new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr i(new pcl::PointIndices);
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setDistanceThreshold(m_params.fittingThreshold); // 使用参数
    
    if (top->size() > 10) {
        seg.setInputCloud(top); seg.segment(*i, *c);
        if ((float)i->indices.size()/top->size() > 0.8) return TYPE_RECTANGLE;
    }
    
    pass.setInputCloud(cloud); pass.setFilterLimits(minPt.y + h*0.2, minPt.y + h*0.8);
    PointCloudT::Ptr mid(new PointCloudT); pass.filter(*mid);
    if (mid->size() > 10) {
        seg.setInputCloud(mid); seg.segment(*i, *c);
        if ((float)i->indices.size()/mid->size() > 0.6) return TYPE_ARCH;
    }
    return TYPE_CIRCLE;
}

void CrossSectionExtractor::processTunnel(ccPointCloud* inputCloud, ccPointCloud*& outputCloud, QString& reportText)
{
    if (!inputCloud) return;

    // 1. CC -> PCL
    PointCloudT::Ptr pclCloud(new PointCloudT);
    unsigned count = inputCloud->size();
    pclCloud->points.resize(count);
    for (unsigned i = 0; i < count; ++i) {
        const CCVector3* P = inputCloud->getPoint(i);
        pclCloud->points[i].x = P->x;
        pclCloud->points[i].y = P->y;
        pclCloud->points[i].z = P->z;
    }

    // 2. 智能处理
    PointCloudT::Ptr clean(new PointCloudT);
    adaptiveProcess(pclCloud, clean);
    alignPerfectly(clean);
    TunnelType type = detectType(clean);

    // 3. 测量
    PointT minPt, maxPt; pcl::getMinMax3D(*clean, minPt, maxPt);
    float span_x = maxPt.x - minPt.x;
    float span_y = maxPt.y - minPt.y;
    float radius = 0.0f;
    
    if (type == TYPE_ARCH || type == TYPE_CIRCLE) {
        pcl::SACSegmentation<PointT> seg;
        seg.setModelType(pcl::SACMODEL_CIRCLE2D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(m_params.fittingThreshold);
        seg.setInputCloud(clean);
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coeff);
        if (!inliers->indices.empty()) radius = coeff->values[2];
    }

    // 4. PCL -> CC (输出点云)
    outputCloud = new ccPointCloud("Processed_Tunnel");
    outputCloud->reserve(clean->size());
    for(const auto& p : clean->points) {
        outputCloud->addPoint(CCVector3(p.x, p.y, p.z));
    }
    outputCloud->showColors(true);
    outputCloud->setColor(ccColor::green);

    // 5. 生成报告
    reportText = QString("检测结果:\n类型: %1\n宽(X): %2 m\n高(Y): %3 m\n拟合半径: %4 m")
                 .arg(QString::fromStdString(getShapeName(type)))
                 .arg(span_x).arg(span_y).arg(radius);
}
