// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

#include <GL/glew.h>

#include "../include/PipeCenterlineExtractor.h"

// CCCoreLib
#include <ccHObjectCaster.h>
#include <ccPointCloud.h>
#include <ccPolyline.h>

// Open3D - required for this plugin
#include <open3d/Open3D.h>

// System
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_map>

PipeCenterlineExtractor::PipeCenterlineExtractor(const Parameters& params)
    : m_params(params)
{
}

bool PipeCenterlineExtractor::extract(ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines)
{
	if (!cloud)
	{
		m_lastError = "Invalid input point cloud";
		return false;
	}

	if (cloud->size() < static_cast<unsigned>(m_params.minPointsPerSegment))
	{
		m_lastError = "Insufficient number of points";
		return false;
	}

	try
	{
		// Use Open3D implementation
		// Step 1: Convert CloudCompare point cloud to Open3D
		auto o3dCloud = ccToOpen3D(cloud);
		if (!o3dCloud || o3dCloud->points_.empty())
		{
			m_lastError = "Failed to convert point cloud to Open3D format";
			return false;
		}

		// Step 2: Preprocess using Open3D
		auto processedCloud = preprocessOpen3D(o3dCloud);
		if (!processedCloud || processedCloud->points_.empty())
		{
			m_lastError = "Failed to preprocess point cloud";
			return false;
		}

		// Step 3: Extract pipe points using Open3D geometry features
		auto pipeCloud = extractPipePointsOpen3D(processedCloud);
		if (!pipeCloud || pipeCloud->points_.empty())
		{
			m_lastError = "No pipe points detected";
			return false;
		}

		// Step 4: Compute centerline using Open3D
		auto centerlinePaths = computeCenterlineOpen3D(pipeCloud);
		if (centerlinePaths.empty())
		{
			m_lastError = "Failed to compute centerline";
			return false;
		}

		// Step 5: Handle branches if enabled
		std::vector<std::vector<Eigen::Vector3d>> finalPaths;
		if (m_params.useBranchDetection)
		{
			finalPaths = detectBranchesOpen3D(centerlinePaths);
		}
		else
		{
			finalPaths = centerlinePaths;
		}

		// Step 6: Smooth paths
		finalPaths = smoothPathsOpen3D(finalPaths);

		// Step 7: Create CloudCompare polylines
		if (!createPolylinesFromPaths(finalPaths, centerlines))
		{
			m_lastError = "Failed to create polylines";
			return false;
		}

		return true;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Exception: %1").arg(e.what());
		return false;
	}
}

std::shared_ptr<open3d::geometry::PointCloud> PipeCenterlineExtractor::ccToOpen3D(ccPointCloud* cloud)
{
	if (!cloud)
	{
		return nullptr;
	}

	auto o3dCloud = std::make_shared<open3d::geometry::PointCloud>();

	// Convert points
	o3dCloud->points_.reserve(cloud->size());
	for (unsigned i = 0; i < cloud->size(); ++i)
	{
		const CCVector3* p = cloud->getPoint(i);
		o3dCloud->points_.push_back(ccToEigen(*p));
	}

	// Convert colors if available
	if (cloud->hasColors())
	{
		o3dCloud->colors_.reserve(cloud->size());
		for (unsigned i = 0; i < cloud->size(); ++i)
		{
			const ccColor::Rgb& color = cloud->getPointColor(i);
			o3dCloud->colors_.push_back(Eigen::Vector3d(color.r / 255.0, color.g / 255.0, color.b / 255.0));
		}
	}

	return o3dCloud;
}

std::shared_ptr<open3d::geometry::PointCloud> PipeCenterlineExtractor::preprocessOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud)
{
	if (!cloud || cloud->points_.empty())
	{
		return nullptr;
	}

	auto processedCloud = std::make_shared<open3d::geometry::PointCloud>(*cloud);

	try
	{
		// Voxel downsampling
		if (m_params.voxelSize > 0.0)
		{
			processedCloud = processedCloud->VoxelDownSample(m_params.voxelSize);
		}

		// Estimate normals if not already available
		if (!processedCloud->HasNormals())
		{
			processedCloud->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
			    m_params.radiusEstimate * 2.0, 30));
		}

		return processedCloud;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Open3D preprocessing failed: %1").arg(e.what());
		return cloud;
	}
}

std::shared_ptr<open3d::geometry::PointCloud> PipeCenterlineExtractor::extractPipePointsOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud)
{
	if (!cloud || cloud->points_.empty())
	{
		return nullptr;
	}

	try
	{
		// Simple geometric filtering based on curvature
		// For now, just return the input cloud
		// In a real implementation, you would use RANSAC or other methods
		return cloud;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Open3D pipe point extraction failed: %1").arg(e.what());
		return nullptr;
	}
}

std::vector<std::vector<Eigen::Vector3d>> PipeCenterlineExtractor::computeCenterlineOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud)
{
	std::vector<std::vector<Eigen::Vector3d>> centerlines;

	if (!cloud || cloud->points_.empty())
	{
		return centerlines;
	}

	try
	{
		// Simple centerline extraction using point cloud skeletonization
		// This is a basic implementation - for production, consider more sophisticated algorithms

		// For now, create a simple line through the centroid of the point cloud
		Eigen::Vector3d centroid(0, 0, 0);
		for (const auto& point : cloud->points_)
		{
			centroid += point;
		}
		centroid /= cloud->points_.size();

		// Create a simple centerline (just one line segment through the centroid)
		std::vector<Eigen::Vector3d> centerline;

		// Find the bounding box
		Eigen::Vector3d min_pt = cloud->points_[0];
		Eigen::Vector3d max_pt = cloud->points_[0];
		for (const auto& point : cloud->points_)
		{
			min_pt = min_pt.cwiseMin(point);
			max_pt = max_pt.cwiseMax(point);
		}

		// Create centerline along the longest axis
		Eigen::Vector3d extent = max_pt - min_pt;
		int             longest_axis;
		extent.maxCoeff(&longest_axis);

		Eigen::Vector3d start = centroid;
		Eigen::Vector3d end   = centroid;
		start[longest_axis]   = min_pt[longest_axis];
		end[longest_axis]     = max_pt[longest_axis];

		centerline.push_back(start);
		centerline.push_back(end);

		centerlines.push_back(centerline);

		return centerlines;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Open3D centerline computation failed: %1").arg(e.what());
		return centerlines;
	}
}

// Simplified medial axis computation using distance from boundary
// This is a basic implementation - for production, consider more sophisticated algorithms

std::vector<std::vector<Eigen::Vector3d>> PipeCenterlineExtractor::detectBranchesOpen3D(const std::vector<std::vector<Eigen::Vector3d>>& centerlines)
{
	if (!m_params.useBranchDetection || centerlines.empty())
	{
		return centerlines;
	}

	// For now, just return the input centerlines
	// Branch detection can be implemented later
	return centerlines;
}

std::vector<std::vector<Eigen::Vector3d>> PipeCenterlineExtractor::smoothPathsOpen3D(const std::vector<std::vector<Eigen::Vector3d>>& paths)
{
	// For now, just return the input paths without smoothing
	// Smoothing can be implemented later if needed
	return paths;
}

bool PipeCenterlineExtractor::createPolylinesFromPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths,
                                                       std::vector<ccPolyline*>&                        polylines)
{
	try
	{
		polylines.clear();

		for (size_t i = 0; i < paths.size(); ++i)
		{
			const auto& path = paths[i];
			if (path.size() < 2)
			{
				continue;
			}

			// Create point cloud for polyline vertices
			ccPointCloud* vertices = new ccPointCloud(QString("vertices_%1").arg(i));
			for (const auto& point : path)
			{
				vertices->addPoint(eigenToCC(point));
			}

			// Create polyline
			ccPolyline* polyline = new ccPolyline(vertices);
			polyline->setName(QString("centerline_%1").arg(i));
			polyline->addPointIndex(0, static_cast<unsigned>(path.size()));
			polyline->setClosed(false);

			polylines.push_back(polyline);
		}

		return !polylines.empty();
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Polyline creation failed: %1").arg(e.what());
		return false;
	}
}

CCVector3 PipeCenterlineExtractor::eigenToCC(const Eigen::Vector3d& eigenVec)
{
	return CCVector3(static_cast<PointCoordinateType>(eigenVec.x()),
	                 static_cast<PointCoordinateType>(eigenVec.y()),
	                 static_cast<PointCoordinateType>(eigenVec.z()));
}

Eigen::Vector3d PipeCenterlineExtractor::ccToEigen(const CCVector3& ccVec)
{
	return Eigen::Vector3d(ccVec.x, ccVec.y, ccVec.z);
}
