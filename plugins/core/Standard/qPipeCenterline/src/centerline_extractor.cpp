#include "centerline_extractor.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cmath>

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

// 计算轨迹长度
static float trackLength(const std::vector<Eigen::Vector3f>& pts) {
    float len = 0.f;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        len += (pts[i] - pts[i - 1]).norm();
    }
    return len;
}

// 简单平滑，保留端点
static void smoothTrack(std::vector<Eigen::Vector3f>& pts, int iterations = 1) {
    if (pts.size() < 3) return;
    for (int it = 0; it < iterations; ++it) {
        std::vector<Eigen::Vector3f> tmp = pts;
        for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
            tmp[i] = 0.25f * pts[i - 1] + 0.5f * pts[i] + 0.25f * pts[i + 1];
        }
        pts.swap(tmp);
    }
}

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
    centerline_tracks_.clear();

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

    // 跟踪参数
    const std::size_t kMinSlicePoints = 10;
    const std::size_t kMinComponentPoints = 6;
    const int kMaxIdleSlices = 5; // 允许更长空洞以跨越缺失段

    std::vector<Track> tracks;

    while (current_x < max_pt.x) {
        // 定义切片范围 [current_x, current_x + resolution]
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(current_x, current_x + slice_resolution);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*slice);

        // 如果切片里点太少，说明是空隙，跳过
        if (slice->size() > kMinSlicePoints) {
            // 估计切片尺度，用于聚类与跟踪
            pcl::PointXYZ s_min, s_max;
            pcl::getMinMax3D(*slice, s_min, s_max);
            float yz_span = std::max(s_max.y - s_min.y, s_max.z - s_min.z);
            float cluster_eps = std::max(slice_resolution * 0.6f, yz_span * 0.3f);
            float cluster_eps_sq = cluster_eps * cluster_eps;

            // 简单基于YZ距离的连通聚类（O(n^2)，切片点数通常不大）
            struct Component {
                std::vector<int> indices;
                float min_y, max_y, min_z, max_z;
            };
            std::vector<int> labels(slice->size(), -1);
            std::vector<Component> components;
            int current_label = 0;

            for (std::size_t i = 0; i < slice->size(); ++i) {
                if (labels[i] != -1) continue;
                Component comp;
                comp.min_y = comp.min_z = std::numeric_limits<float>::max();
                comp.max_y = comp.max_z = -std::numeric_limits<float>::max();

                std::vector<std::size_t> stack;
                stack.push_back(i);
                labels[i] = current_label;

                while (!stack.empty()) {
                    std::size_t idx = stack.back();
                    stack.pop_back();
                    const auto& p = slice->points[idx];
                    comp.indices.push_back(static_cast<int>(idx));
                    comp.min_y = std::min(comp.min_y, p.y);
                    comp.max_y = std::max(comp.max_y, p.y);
                    comp.min_z = std::min(comp.min_z, p.z);
                    comp.max_z = std::max(comp.max_z, p.z);

                    for (std::size_t j = 0; j < slice->size(); ++j) {
                        if (labels[j] != -1) continue;
                        const auto& q = slice->points[j];
                        float dy = q.y - p.y;
                        float dz = q.z - p.z;
                        if (dy * dy + dz * dz <= cluster_eps_sq) {
                            labels[j] = current_label;
                            stack.push_back(j);
                        }
                    }
                }
                components.push_back(comp);
                ++current_label;
            }

            // 计算每个分量的中心
            struct CompCenter {
                Eigen::Vector2f yz;
                Component* comp;
            };
            std::vector<CompCenter> centers;
            centers.reserve(components.size());
            for (auto& comp : components) {
                if (comp.indices.size() < kMinComponentPoints) {
                    continue;
                }
                float cy = (comp.min_y + comp.max_y) * 0.5f;
                float cz = (comp.min_z + comp.max_z) * 0.5f;
                centers.push_back({Eigen::Vector2f(cy, cz), &comp});
            }

            // 跟踪匹配
            float follow_radius = std::max(cluster_eps * 2.0f, yz_span * 0.8f);
            float follow_radius_sq = follow_radius * follow_radius;
            std::vector<bool> center_used(centers.size(), false);

            for (auto& track : tracks) {
                if (!track.active) continue;
                float best_dist = follow_radius_sq;
                int best_idx = -1;
                for (std::size_t ci = 0; ci < centers.size(); ++ci) {
                    if (center_used[ci]) continue;
                    float dy = centers[ci].yz.x() - track.lastYZ.x();
                    float dz = centers[ci].yz.y() - track.lastYZ.y();
                    float dist2 = dy * dy + dz * dz;
                    if (dist2 < best_dist) {
                        best_dist = dist2;
                        best_idx = static_cast<int>(ci);
                    }
                }

                if (best_idx >= 0) {
                    center_used[best_idx] = true;
                    // 桥接缺失段：若空洞存在，插入一个中间点
                    Eigen::Vector3f local_center(current_x + slice_resolution / 2.0f,
                                                 centers[best_idx].yz.x(),
                                                 centers[best_idx].yz.y());
                    Eigen::Vector3f world_center = inverse_transform * local_center;
                    if (track.idle > 0 && !track.nodes.empty()) {
                        Eigen::Vector3f mid = 0.5f * (track.nodes.back() + world_center);
                        track.nodes.push_back(mid);
                    }
                    track.idle = 0;
                    track.lastYZ = centers[best_idx].yz;
                    track.nodes.push_back(world_center);
                } else {
                    track.idle++;
                    if (track.idle > kMaxIdleSlices) {
                        track.active = false;
                    }
                }
            }

            // 为未被匹配的分量创建新分支
            for (std::size_t ci = 0; ci < centers.size(); ++ci) {
                if (center_used[ci]) continue;
                Track t;
                t.lastYZ = centers[ci].yz;
                t.idle = 0;
                t.active = true;
                Eigen::Vector3f local_center(current_x + slice_resolution / 2.0f,
                                             centers[ci].yz.x(),
                                             centers[ci].yz.y());
                Eigen::Vector3f world_center = inverse_transform * local_center;
                // 尝试把新分支连接到最近的已有分支末端，减少分叉断裂
                float attach_radius_sq = follow_radius_sq;
                bool attached = false;
                Eigen::Vector3f anchor;
                float best_attach = attach_radius_sq;
                for (const auto& ex : tracks) {
                    if (ex.nodes.empty()) continue;
                    const Eigen::Vector3f& tail = ex.nodes.back();
                    float d2 = (tail - world_center).squaredNorm();
                    if (d2 < best_attach) {
                        best_attach = d2;
                        anchor = tail;
                        attached = true;
                    }
                }
                if (attached) {
                    t.nodes.push_back(anchor);
                    // 如果距离明显大于采样步长，插入中点让折线更平滑
                    if (best_attach > slice_resolution * slice_resolution) {
                        t.nodes.push_back(0.5f * (anchor + world_center));
                    }
                }
                t.nodes.push_back(world_center);
                tracks.push_back(std::move(t));
            }
        } else {
            // 该切片无效，所有活动分支 idle+1
            for (auto& track : tracks) {
                if (!track.active) continue;
                track.idle++;
                if (track.idle > kMaxIdleSlices) {
                    track.active = false;
                }
            }
        }

        current_x += slice_resolution;
    }

    // 收集有效分支（至少两个节点）
    std::size_t longest = 0;
    for (const auto& t : tracks) {
        if (t.nodes.size() >= 2) {
            centerline_tracks_.push_back(t.nodes);
            if (t.nodes.size() > longest) {
                longest = t.nodes.size();
                centerline_points_ = t.nodes; // 兼容旧接口，主干取最长
            }
        }
    }

    // 若没有分支满足要求，尝试保留单节点最长的（避免完全空）
    if (centerline_tracks_.empty()) {
        for (const auto& t : tracks) {
            if (!t.nodes.empty()) {
                centerline_tracks_.push_back(t.nodes);
                if (t.nodes.size() > centerline_points_.size()) {
                    centerline_points_ = t.nodes;
                }
            }
        }
    }

    // 后处理：滤掉过短分支并平滑
    const float min_track_length = std::max(slice_resolution * 4.0f, 0.1f);
    std::vector<std::vector<Eigen::Vector3f>> cleaned;
    cleaned.reserve(centerline_tracks_.size());
    for (auto branch : centerline_tracks_) {
        if (branch.size() < 2) {
            continue;
        }
        if (trackLength(branch) < min_track_length) {
            continue; // 去掉很短的噪声分支
        }
        smoothTrack(branch, 2);
        cleaned.push_back(std::move(branch));
    }
    centerline_tracks_.swap(cleaned);

    // 更新兼容主干（取最长的平滑后结果）
    longest = 0;
    for (const auto& b : centerline_tracks_) {
        if (b.size() > longest) {
            longest = b.size();
            centerline_points_ = b;
        }
    }

    std::cout << "[Algo] 3. Extraction complete. Generated " << centerline_points_.size() << " nodes." << std::endl;
}

// 计算切片的几何中心
// 这里实现了你说的 "最小外接圆/矩形中心" 的逻辑，并增加了主干跟踪
// 如果提供了 prevCenterYZ，则优先在其附近的点集上取中心，避免分叉时跳向其他支路
Eigen::Vector3f CenterlineExtractor::computeSliceCenter(
    pcl::PointCloud<pcl::PointXYZ>::Ptr slice_cloud,
    const Eigen::Vector2f* prevCenterYZ,
    float followRadiusSq,
    std::size_t minFollowPoints) {
    pcl::PointXYZ min_p, max_p;
    pcl::getMinMax3D(*slice_cloud, min_p, max_p);

    // 默认：整个切片的中心
    float center_y = (min_p.y + max_p.y) / 2.0f;
    float center_z = (min_p.z + max_p.z) / 2.0f;

    // 若有上一截中心，则尝试在其邻域内计算中心，避免分叉偏移
    if (prevCenterYZ && !slice_cloud->empty()) {
        float local_min_y = std::numeric_limits<float>::max();
        float local_max_y = -std::numeric_limits<float>::max();
        float local_min_z = std::numeric_limits<float>::max();
        float local_max_z = -std::numeric_limits<float>::max();
        std::size_t cnt = 0;

        float best_dist = std::numeric_limits<float>::max();
        pcl::PointXYZ best_pt;

        for (const auto& p : slice_cloud->points) {
            float dy = p.y - prevCenterYZ->x();
            float dz = p.z - prevCenterYZ->y();
            float dist2 = dy * dy + dz * dz;

            if (dist2 < best_dist) {
                best_dist = dist2;
                best_pt = p;
            }

            if (dist2 <= followRadiusSq) {
                local_min_y = std::min(local_min_y, p.y);
                local_max_y = std::max(local_max_y, p.y);
                local_min_z = std::min(local_min_z, p.z);
                local_max_z = std::max(local_max_z, p.z);
                ++cnt;
            }
        }

        if (cnt >= minFollowPoints) {
            center_y = (local_min_y + local_max_y) / 2.0f;
            center_z = (local_min_z + local_max_z) / 2.0f;
        } else {
            // 降级：使用与上一中心最近的点，保持连续性
            center_y = best_pt.y;
            center_z = best_pt.z;
        }
    }

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
