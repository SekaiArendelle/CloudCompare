#include "qPipeSurfaceRecon.h"

// Qt Headers
#include <QProgressDialog>
#include <QApplication>
#include <QtConcurrentRun>
#include <QFuture>
#include <QMessageBox>
#include <QMainWindow>
#include <QFileDialog> 
#include <QImage>        
#include <QColor>        

// 系统头文件
#ifndef CC_WINDOWS
#include <unistd.h>
#endif

// CC Headers
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccLog.h>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// VTK Headers
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkMath.h>
#include <vtkClipPolyData.h>
#include <vtkLinearSubdivisionFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkDataArray.h>

// =========================================================================
//                             参数配置
// =========================================================================
const double TUNNEL_WIDTH = 2.0;
const double WALL_HEIGHT = 1.0;
const double ARCH_HEIGHT = 0.5;

// 分辨率配置：确保足够高以显示纹理细节
const int ARCH_RESOLUTION = 40; 
const int FLOOR_RESOLUTION = 20; 
const int WALL_RESOLUTION = 10;  

const double RESAMPLE_STEP = 0.05; 
const double JUMP_THRESHOLD = 3.0; 
const double WALL_THICKNESS = 0.01; 
const double TEXTURE_SCALE_U = 5.0; 

// =========================================================================
//                             数据结构与辅助类
// =========================================================================

// [关键修改] 定义一个纯数据结构，不依赖 QImage，确保线程安全
struct RawTextureData {
    int width = 0;
    int height = 0;
    std::vector<unsigned char> pixels; // 存储 RGBRGB...

    bool isValid() const { return width > 0 && height > 0 && !pixels.empty(); }
};

struct LocalFrame {
    double right[3];
    double up[3];
    double tangent[3];
};

// [关键修改] 重写采样函数，直接读取字节数组
void SampleTextureRaw(const RawTextureData& tex, double u, double v, unsigned char rgb[3], const unsigned char defaultColor[3]) {
    if (!tex.isValid()) {
        rgb[0] = defaultColor[0]; rgb[1] = defaultColor[1]; rgb[2] = defaultColor[2]; 
        return;
    }

    // U 方向平铺 (Tiling)
    double u_img = u / TEXTURE_SCALE_U;
    u_img = u_img - std::floor(u_img); 

    // V 方向钳制 (Clamp)
    double v_img = v;
    if (v_img < 0.0) v_img = 0.0;
    if (v_img > 1.0) v_img = 1.0;

    // 计算像素坐标
    int x = static_cast<int>(u_img * (tex.width - 1));
    int y = static_cast<int>((1.0 - v_img) * (tex.height - 1)); // V轴反转，因为图片原点通常在左上

    // 边界检查
    if (x < 0) x = 0; if (x >= tex.width) x = tex.width - 1;
    if (y < 0) y = 0; if (y >= tex.height) y = tex.height - 1;

    // 直接从数组读取 RGB
    size_t index = (y * tex.width + x) * 3;
    if (index + 2 < tex.pixels.size()) {
        rgb[0] = tex.pixels[index + 0];
        rgb[1] = tex.pixels[index + 1];
        rgb[2] = tex.pixels[index + 2];
    } else {
        rgb[0] = defaultColor[0]; rgb[1] = defaultColor[1]; rgb[2] = defaultColor[2];
    }
}

class PathProcessor {
public:
    static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> SplitByDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segments;
        if (!cloud || cloud->empty()) return segments;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_seg(new pcl::PointCloud<pcl::PointXYZ>);
        current_seg->push_back(cloud->points[0]);
        for (size_t i = 0; i < cloud->size() - 1; ++i) {
            pcl::PointXYZ p_next = cloud->points[i+1];
            pcl::PointXYZ p_curr = cloud->points[i];
            double dist = std::sqrt(std::pow(p_next.x - p_curr.x, 2) + std::pow(p_next.y - p_curr.y, 2) + std::pow(p_next.z - p_curr.z, 2));
            if (dist > JUMP_THRESHOLD) {
                if (current_seg->size() > 1) segments.push_back(current_seg);
                current_seg = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
                current_seg->push_back(p_next);
            } else {
                current_seg->push_back(p_next);
            }
        }
        if (current_seg->size() > 1) segments.push_back(current_seg);
        return segments;
    }

    static pcl::PointCloud<pcl::PointXYZ>::Ptr ResamplePath(pcl::PointCloud<pcl::PointXYZ>::Ptr path, double step) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr uniform_path(new pcl::PointCloud<pcl::PointXYZ>);
        if (!path || path->size() < 2) return path ? path : uniform_path;
        
        uniform_path->push_back(path->points[0]);
        double current_dist = 0.0;
        for (size_t i = 0; i < path->points.size() - 1; ++i) {
            pcl::PointXYZ p1 = path->points[i];
            pcl::PointXYZ p2 = path->points[i+1];
            double seg_len = std::sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
            if (seg_len < 1e-6) continue;
            double dist_walked = 0.0;
            while (current_dist + (seg_len - dist_walked) >= step) {
                double needed = step - current_dist;
                double ratio = (dist_walked + needed) / seg_len;
                pcl::PointXYZ new_pt;
                new_pt.x = p1.x + ratio * (p2.x - p1.x);
                new_pt.y = p1.y + ratio * (p2.y - p1.y);
                new_pt.z = p1.z + ratio * (p2.z - p1.z);
                uniform_path->push_back(new_pt);
                current_dist = 0.0;
                dist_walked += needed;
            }
            current_dist += (seg_len - dist_walked);
        }
        return uniform_path;
    }

    static LocalFrame ComputeFrame(pcl::PointCloud<pcl::PointXYZ>::Ptr path, size_t i) {
        LocalFrame f = {{1,0,0}, {0,1,0}, {1,0,0}}; 
        if (!path || path->empty()) return f;
        if (i >= path->points.size()) return f;

        pcl::PointXYZ p = path->points[i];
        if (i < path->points.size() - 1) {
            f.tangent[0] = path->points[i+1].x - p.x; f.tangent[1] = path->points[i+1].y - p.y; f.tangent[2] = path->points[i+1].z - p.z;
        } else if (i > 0) {
            f.tangent[0] = p.x - path->points[i-1].x; f.tangent[1] = p.y - path->points[i-1].y; f.tangent[2] = p.z - path->points[i-1].z;
        } else {
            f.tangent[0] = 1; f.tangent[1] = 0; f.tangent[2] = 0;
        }
        if (vtkMath::Norm(f.tangent) < 1e-6) { f.tangent[0] = 1; f.tangent[1] = 0; f.tangent[2] = 0; }
        vtkMath::Normalize(f.tangent);
        double globZ[3] = {0,0,1};
        vtkMath::Cross(f.tangent, globZ, f.right);
        if (vtkMath::Norm(f.right) < 0.01) { f.right[0]=1; f.right[1]=0; f.right[2]=0; }
        vtkMath::Normalize(f.right);
        vtkMath::Cross(f.right, f.tangent, f.up);
        vtkMath::Normalize(f.up);
        return f;
    }
};

// =========================================================================
//                             核心生成与裁切逻辑
// =========================================================================

void GenerateTunnelMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr path, 
                        vtkPolyData* floorPoly, 
                        vtkPolyData* wallPoly, 
                        vtkPolyData* roofPoly,
                        const RawTextureData& texFloor, // [修改] 接收 RawData
                        const RawTextureData& texWall,  // [修改] 接收 RawData
                        const RawTextureData& texRoof,  // [修改] 接收 RawData
                        bool useTexture) {

    if (!path || path->empty()) return;

    vtkSmartPointer<vtkPoints> floorPts = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> wallPts = vtkSmartPointer<vtkPoints>::New(); 
    vtkSmartPointer<vtkPoints> roofPts = vtkSmartPointer<vtkPoints>::New();

    vtkSmartPointer<vtkCellArray> floorCells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkCellArray> wallCells = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkCellArray> roofCells = vtkSmartPointer<vtkCellArray>::New();

    vtkSmartPointer<vtkUnsignedCharArray> floorColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> wallColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> roofColors = vtkSmartPointer<vtkUnsignedCharArray>::New();

    floorColors->SetNumberOfComponents(3); floorColors->SetName("Colors");
    wallColors->SetNumberOfComponents(3); wallColors->SetName("Colors");
    roofColors->SetNumberOfComponents(3); roofColors->SetName("Colors");

    // [调试] 设置极端的默认颜色，方便一眼看出是否加载失败
    // 如果你看到纯红或纯绿，说明图片加载失败
    unsigned char defFloor[3] = {255, 0, 0};   // 红色默认
    unsigned char defWall[3]  = {0, 255, 0};   // 绿色默认
    unsigned char defRoof[3]  = {0, 0, 255};   // 蓝色默认

    double accumulated_dist = 0.0; 

    // 临时存储墙面数据
    struct VertexData { double x,y,z; unsigned char r,g,b; };
    std::vector<VertexData> leftWallBuffer;
    std::vector<VertexData> rightWallBuffer;

    for (size_t i = 0; i < path->points.size(); ++i) {
        pcl::PointXYZ p = path->points[i];
        if (i > 0) {
            pcl::PointXYZ prev = path->points[i-1];
            accumulated_dist += std::sqrt(std::pow(p.x-prev.x, 2) + std::pow(p.y-prev.y, 2) + std::pow(p.z-prev.z, 2));
        }

        LocalFrame frame = PathProcessor::ComputeFrame(path, i);
        double hw = TUNNEL_WIDTH / 2.0;
        unsigned char rgb[3];

        // --- 1. Floor Generation ---
        for (int k = 0; k <= FLOOR_RESOLUTION; ++k) {
            double ratio = (double)k / FLOOR_RESOLUTION; 
            double u_local = hw * (1.0 - 2.0 * ratio); // Right to Left
            
            double fx = p.x + u_local * frame.right[0];
            double fy = p.y + u_local * frame.right[1];
            double fz = p.z + u_local * frame.right[2];
            
            floorPts->InsertNextPoint(fx, fy, fz);
            
            if (useTexture) SampleTextureRaw(texFloor, accumulated_dist, ratio, rgb, defFloor);
            else { rgb[0]=128; rgb[1]=128; rgb[2]=128; }
            floorColors->InsertNextTypedTuple(rgb);
        }

        // --- 2. Wall Generation ---
        // Right Wall
        for (int k = 0; k <= WALL_RESOLUTION; ++k) {
            double ratio = (double)k / WALL_RESOLUTION; 
            double h = WALL_HEIGHT * ratio;
            
            double wx = p.x + hw * frame.right[0] + h * frame.up[0];
            double wy = p.y + hw * frame.right[1] + h * frame.up[1];
            double wz = p.z + hw * frame.right[2] + h * frame.up[2];
            
            if (useTexture) SampleTextureRaw(texWall, accumulated_dist, ratio, rgb, defWall);
            else { rgb[0]=128; rgb[1]=128; rgb[2]=128; }
            
            rightWallBuffer.push_back({wx, wy, wz, rgb[0], rgb[1], rgb[2]});
        }

        // Left Wall
        for (int k = 0; k <= WALL_RESOLUTION; ++k) {
            double ratio = (double)k / WALL_RESOLUTION; 
            double h = WALL_HEIGHT * ratio;
            
            double wx = p.x - hw * frame.right[0] + h * frame.up[0];
            double wy = p.y - hw * frame.right[1] + h * frame.up[1];
            double wz = p.z - hw * frame.right[2] + h * frame.up[2];
            
            if (useTexture) SampleTextureRaw(texWall, accumulated_dist, ratio, rgb, defWall);
            else { rgb[0]=128; rgb[1]=128; rgb[2]=128; }

            leftWallBuffer.push_back({wx, wy, wz, rgb[0], rgb[1], rgb[2]});
        }

        // --- 3. Roof Generation ---
        for (int k = 0; k <= ARCH_RESOLUTION; ++k) {
            double t = (double)k / ARCH_RESOLUTION * M_PI;
            double u_local = hw * cos(t);
            double v_local = WALL_HEIGHT + ARCH_HEIGHT * sin(t);
            double rx = p.x + u_local * frame.right[0] + v_local * frame.up[0];
            double ry = p.y + u_local * frame.right[1] + v_local * frame.up[1];
            double rz = p.z + u_local * frame.right[2] + v_local * frame.up[2];
            roofPts->InsertNextPoint(rx, ry, rz);
            
            if (useTexture) {
                double v_ratio = (double)k / ARCH_RESOLUTION;
                SampleTextureRaw(texRoof, accumulated_dist, v_ratio, rgb, defRoof);
            } else { rgb[0]=128; rgb[1]=128; rgb[2]=128; }
            roofColors->InsertNextTypedTuple(rgb);
        }
    }

    // 写入墙面点
    for(const auto& v : rightWallBuffer) {
        wallPts->InsertNextPoint(v.x, v.y, v.z);
        unsigned char c[3] = {v.r, v.g, v.b};
        wallColors->InsertNextTypedTuple(c);
    }
    for(const auto& v : leftWallBuffer) {
        wallPts->InsertNextPoint(v.x, v.y, v.z);
        unsigned char c[3] = {v.r, v.g, v.b};
        wallColors->InsertNextTypedTuple(c);
    }

    // 三角网构建
    auto AddStrip = [](vtkCellArray* polys, int ptsPerRing, int nRings, int startOffset) {
        if (nRings < 2) return; 
        for (int i = 0; i < nRings - 1; ++i) {
            int r1 = startOffset + i * ptsPerRing;
            int r2 = startOffset + (i + 1) * ptsPerRing;
            for (int j = 0; j < ptsPerRing - 1; ++j) {
                vtkSmartPointer<vtkIdList> tri1 = vtkSmartPointer<vtkIdList>::New();
                tri1->InsertNextId(r1 + j); tri1->InsertNextId(r2 + j + 1); tri1->InsertNextId(r1 + j + 1);
                polys->InsertNextCell(tri1);
                vtkSmartPointer<vtkIdList> tri2 = vtkSmartPointer<vtkIdList>::New();
                tri2->InsertNextId(r1 + j); tri2->InsertNextId(r2 + j); tri2->InsertNextId(r2 + j + 1);
                polys->InsertNextCell(tri2);
            }
        }
    };
    
    int pathSize = static_cast<int>(path->points.size());
    AddStrip(floorCells, FLOOR_RESOLUTION + 1, pathSize, 0);
    AddStrip(wallCells, WALL_RESOLUTION + 1, pathSize, 0); // Right Wall
    int rightWallTotalPts = pathSize * (WALL_RESOLUTION + 1);
    AddStrip(wallCells, WALL_RESOLUTION + 1, pathSize, rightWallTotalPts); // Left Wall
    AddStrip(roofCells, ARCH_RESOLUTION + 1, pathSize, 0);

    floorPoly->SetPoints(floorPts); floorPoly->SetPolys(floorCells);
    floorPoly->GetPointData()->SetScalars(floorColors);

    wallPoly->SetPoints(wallPts);   wallPoly->SetPolys(wallCells);
    wallPoly->GetPointData()->SetScalars(wallColors);

    roofPoly->SetPoints(roofPts);   roofPoly->SetPolys(roofCells);
    roofPoly->GetPointData()->SetScalars(roofColors);
}

void ClipMeshByArchProfile(vtkPolyData* mesh, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, const std::vector<LocalFrame>& global_frames, bool isFloor) {
    if (!mesh || mesh->GetNumberOfPoints() == 0) return;
    
    // 使用简单的裁切，不再细分，因为原始网格已经很密
    vtkSmartPointer<vtkFloatArray> scalars = vtkSmartPointer<vtkFloatArray>::New();
    scalars->SetName("KeepOrKill");
    scalars->SetNumberOfTuples(mesh->GetNumberOfPoints());
    vtkPoints* pts = mesh->GetPoints();
    
    double inner_half_width = (TUNNEL_WIDTH / 2.0) - WALL_THICKNESS;
    double inner_arch_height_radius = ARCH_HEIGHT - WALL_THICKNESS; 
    if (inner_arch_height_radius < 0.1) inner_arch_height_radius = 0.1;

    std::vector<int> idx; std::vector<float> sq_dist;
    for (vtkIdType i = 0; i < pts->GetNumberOfPoints(); ++i) {
        double value = 1.0; 
        if (isFloor) {
            scalars->SetValue(i, 1.0); continue;
        }
        double p[3]; pts->GetPoint(i, p);
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) { scalars->SetValue(i, -1.0); continue; }
        pcl::PointXYZ queryPt(p[0], p[1], p[2]);
        idx.clear(); sq_dist.clear();
        int found = kdtree.nearestKSearch(queryPt, 1, idx, sq_dist);
        if (found > 0 && !idx.empty() && idx[0] >= 0 && static_cast<size_t>(idx[0]) < global_frames.size()) {
            pcl::PointXYZ centerPt = kdtree.getInputCloud()->points[idx[0]];
            LocalFrame frame = global_frames[idx[0]];
            double dx = queryPt.x - centerPt.x; double dy = queryPt.y - centerPt.y; double dz = queryPt.z - centerPt.z;
            double dist_u = std::abs(dx*frame.right[0] + dy*frame.right[1] + dz*frame.right[2]);
            double dist_v = dx*frame.up[0] + dy*frame.up[1] + dz*frame.up[2];
            if (dist_v > -0.5) { 
                if (dist_v < WALL_HEIGHT) { if (dist_u < inner_half_width) value = -1.0; }
                else {
                    double local_v = dist_v - WALL_HEIGHT;
                    double term_u = (dist_u * dist_u) / (inner_half_width * inner_half_width);
                    double term_v = (local_v * local_v) / (inner_arch_height_radius * inner_arch_height_radius);
                    if (term_u + term_v < 1.0) value = -1.0; 
                }
            }
        } 
        scalars->SetValue(i, value);
    }
    
    mesh->GetPointData()->AddArray(scalars);
    mesh->GetPointData()->SetActiveScalars("KeepOrKill"); 

    vtkSmartPointer<vtkClipPolyData> clipper = vtkSmartPointer<vtkClipPolyData>::New();
    clipper->SetInputData(mesh);
    clipper->SetValue(0.0);
    clipper->SetInsideOut(false);
    clipper->Update();
    
    mesh->DeepCopy(clipper->GetOutput());

    if (mesh->GetPointData()->HasArray("Colors")) {
        mesh->GetPointData()->SetActiveScalars("Colors");
    }
}

// =========================================================================
//                             CC <-> PCL/VTK 转换工具
// =========================================================================

pcl::PointCloud<pcl::PointXYZ>::Ptr ccToPCL(ccPointCloud* ccCloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (!ccCloud || ccCloud->size() == 0) return pclCloud;
    unsigned pointCount = ccCloud->size();
    pclCloud->width = pointCount; pclCloud->height = 1; pclCloud->is_dense = false;
    pclCloud->points.resize(pointCount);
    for (unsigned i = 0; i < pointCount; ++i) {
        const CCVector3* P = ccCloud->getPoint(i);
        pclCloud->points[i].x = static_cast<float>(P->x);
        pclCloud->points[i].y = static_cast<float>(P->y);
        pclCloud->points[i].z = static_cast<float>(P->z);
    }
    return pclCloud;
}

ccMesh* vtkToCC(vtkPolyData* vtkMesh, const QString& name) {
    if (!vtkMesh || vtkMesh->GetNumberOfPoints() == 0) return nullptr;
    vtkPoints* vtkPts = vtkMesh->GetPoints();
    vtkIdType numPts = vtkPts->GetNumberOfPoints();
    
    vtkDataArray* colors = nullptr;
    if (vtkMesh->GetPointData()->HasArray("Colors")) {
        colors = vtkMesh->GetPointData()->GetArray("Colors");
    } else if (vtkMesh->GetPointData()->GetScalars()) {
        colors = vtkMesh->GetPointData()->GetScalars();
    }

    ccPointCloud* vertices = new ccPointCloud(name + ".vertices");
    vertices->reserve(numPts);

    bool hasColor = (colors != nullptr && colors->GetNumberOfComponents() == 3);
    
    for(vtkIdType i=0; i<numPts; ++i) {
        double p[3]; vtkPts->GetPoint(i, p);
        vertices->addPoint(CCVector3(static_cast<PointCoordinateType>(p[0]), 
                                     static_cast<PointCoordinateType>(p[1]), 
                                     static_cast<PointCoordinateType>(p[2])));
    }

    if (hasColor) {
        if(!vertices->resizeTheRGBTable(false)) {
            ccLog::Warning("[vtkToCC] Failed to allocate RGB table");
            hasColor = false;
        } else {
            for(vtkIdType i=0; i<numPts; ++i) {
                double rgb[3];
                colors->GetTuple(i, rgb); 
                vertices->setPointColor(i, ccColor::Rgb(static_cast<unsigned char>(rgb[0]), 
                                                        static_cast<unsigned char>(rgb[1]), 
                                                        static_cast<unsigned char>(rgb[2])));
            }
            vertices->showColors(true);
        }
    }

    ccMesh* mesh = new ccMesh(vertices);
    mesh->setName(name);
    vtkCellArray* polys = vtkMesh->GetPolys();
    mesh->reserve(polys->GetNumberOfCells());
    polys->InitTraversal();
    vtkIdType npts; const vtkIdType* pts;
    while(polys->GetNextCell(npts, pts)) {
        if (npts == 3) mesh->addTriangle(pts[0], pts[1], pts[2]);
    }
    
    vertices->setEnabled(false);
    mesh->setVisible(true);
    if (hasColor) mesh->showColors(true);
    
    return mesh;
}

// =========================================================================
//                             插件类实现
// =========================================================================

qPipeSurfaceRecon::qPipeSurfaceRecon(QObject* parent)
    : QObject(parent), ccStdPluginInterface(":/CC/plugin/qPipeSurfaceRecon/info.json"), m_action(nullptr) {
}

void qPipeSurfaceRecon::onNewSelection(const ccHObject::Container& selectedEntities) {
    if (m_action) {
        bool enable = (selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
        m_action->setEnabled(enable);
    }
}

QList<QAction *> qPipeSurfaceRecon::getActions() {
    if (!m_action) {
        m_action = new QAction(getName(), this);
        m_action->setToolTip("Generate Optimized Tunnel Mesh from Centerline");
        m_action->setIcon(getIcon()); 
        connect(m_action, &QAction::triggered, this, &qPipeSurfaceRecon::doAction);
    }
    return QList<QAction *>{ m_action };
}

struct MeshResult {
    vtkPolyData* f;
    vtkPolyData* w;
    vtkPolyData* r;
};

// [关键修改] 辅助函数：将 QImage 转换为线程安全的 RawTextureData
RawTextureData ConvertToRaw(const QImage& img) {
    RawTextureData raw;
    if (img.isNull()) return raw;
    
    // 强制转换为 RGB888，确保每像素3字节，无 padding 烦恼
    QImage rgbImg = img.convertToFormat(QImage::Format_RGB888);
    raw.width = rgbImg.width();
    raw.height = rgbImg.height();
    
    // 复制数据到 std::vector
    const unsigned char* bits = rgbImg.bits();
    // QImage 的行可能会有 padding (stride)，所以按行复制最安全
    raw.pixels.reserve(raw.width * raw.height * 3);
    for (int y = 0; y < raw.height; ++y) {
        const unsigned char* line = rgbImg.scanLine(y);
        for (int x = 0; x < raw.width * 3; ++x) {
            raw.pixels.push_back(line[x]);
        }
    }
    return raw;
}

void qPipeSurfaceRecon::doAction() {
    if (!m_app || !m_app->haveOneSelection()) return;
    ccHObject* ent = m_app->getSelectedEntities()[0];
    if (!ent->isA(CC_TYPES::POINT_CLOUD)) return;
    
    ccPointCloud* inputCC = static_cast<ccPointCloud*>(ent);
    if (inputCC->size() == 0) return;

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(nullptr, "Texture Mapping", 
                                  "Apply texture from images?\n(Yes: Select 3 images for Floor/Wall/Roof, No: Geometry Only)",
                                  QMessageBox::Yes|QMessageBox::No);

    bool useTexture = false;
    
    // [关键修改] 使用 RawTextureData 替代 QImage
    RawTextureData rawFloor, rawWall, rawRoof;

    if (reply == QMessageBox::Yes) {
        auto loadImg = [&](QString title) -> QImage {
            QString path = QFileDialog::getOpenFileName(nullptr, title, QString(), "Images (*.png *.jpg *.jpeg *.bmp)");
            if (path.isEmpty()) return QImage(); 
            return QImage(path);
        };

        QImage imgF = loadImg("Select 1/3: FLOOR Texture");
        QImage imgW = loadImg("Select 2/3: WALL Texture");
        QImage imgR = loadImg("Select 3/3: ROOF Texture");

        if (!imgF.isNull() || !imgW.isNull() || !imgR.isNull()) {
            useTexture = true;
            // 在主线程完成转换，确保数据完全独立
            rawFloor = ConvertToRaw(imgF);
            rawWall  = ConvertToRaw(imgW);
            rawRoof  = ConvertToRaw(imgR);
            m_app->dispToConsole("Images converted to raw buffers for thread safety.", ccMainAppInterface::STD_CONSOLE_MESSAGE);
        }
    }

    QWidget* parent = m_app->getMainWindow() ? static_cast<QWidget*>(m_app->getMainWindow()) : nullptr;
    QProgressDialog pDlg("Generating Tunnel Mesh...", "Cancel", 0, 0, parent);
    pDlg.setWindowModality(Qt::WindowModal);
    pDlg.show();

    // 传递 raw 数据到 Lambda，按值捕获 (RawTextureData 内部是 vector，拷贝是深拷贝，安全)
    QFuture<std::vector<MeshResult>> future = QtConcurrent::run([=]() {
        std::vector<MeshResult> results;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = ccToPCL(inputCC);
        auto raw_segments = PathProcessor::SplitByDistance(cloud);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_path_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> resampled_segments;
        std::vector<LocalFrame> global_frames;

        for (auto& seg : raw_segments) {
            auto p = PathProcessor::ResamplePath(seg, RESAMPLE_STEP);
            if (p && p->size() > 1) {
                resampled_segments.push_back(p);
                for(size_t i=0; i<p->size(); ++i) {
                    global_path_cloud->push_back(p->points[i]);
                    global_frames.push_back(PathProcessor::ComputeFrame(p, i));
                }
            }
        }

        if(global_path_cloud->empty()) return results;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(global_path_cloud);

        for (auto& path : resampled_segments) {
            vtkPolyData* f = vtkPolyData::New();
            vtkPolyData* w = vtkPolyData::New();
            vtkPolyData* r = vtkPolyData::New();
            
            // 使用 RawData 进行生成
            GenerateTunnelMesh(path, f, w, r, rawFloor, rawWall, rawRoof, useTexture);
            
            ClipMeshByArchProfile(f, kdtree, global_frames, true);
            ClipMeshByArchProfile(w, kdtree, global_frames, false);
            ClipMeshByArchProfile(r, kdtree, global_frames, false);

            results.push_back({f, w, r});
        }
        return results;
    });

    while (!future.isFinished()) {
        QApplication::processEvents();
#if defined(CC_WINDOWS)
        ::Sleep(100);
#else
        usleep(100 * 1000);
#endif
    }
    pDlg.close();

    std::vector<MeshResult> meshes = future.result();
    if (meshes.empty()) {
        m_app->dispToConsole("Generation failed.", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
    }

    ccHObject* group = new ccHObject(useTexture ? "Textured_Tunnel_Model" : "Geometry_Tunnel_Model");
    int segCount = 0;
    
    for (auto& res : meshes) {
        QString baseName = QString("Seg_%1").arg(segCount++);
        auto ProcessPart = [&](vtkPolyData* poly, QString suffix) {
            if (poly->GetNumberOfPoints() > 0) {
                ccMesh* ccM = vtkToCC(poly, baseName + suffix);
                if (ccM) {
                    group->addChild(ccM);
                    ccM->setDisplay(inputCC->getDisplay()); 
                    if (ccM->hasColors()) ccM->showColors(true);
                }
            }
            poly->Delete(); 
        };
        ProcessPart(res.f, "_Floor");
        ProcessPart(res.w, "_Wall");
        ProcessPart(res.r, "_Roof");
    }

    m_app->addToDB(group);
    m_app->refreshAll();
    m_app->dispToConsole("Tunnel generation completed!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
}
