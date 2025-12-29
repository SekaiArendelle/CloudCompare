// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

#include "../include/PipeCenterlineExtractor.h"

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <ccHObjectCaster.h>

// Conditionally include Open3D
#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
#include <open3d/Open3D.h>
#endif

// System
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <queue>

PipeCenterlineExtractor::PipeCenterlineExtractor( const Parameters& params )
	: m_params( params )
{
}

bool PipeCenterlineExtractor::extract( ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines )
{
	if ( !cloud )
	{
		m_lastError = "Invalid input point cloud";
		return false;
	}
	
	if ( cloud->size() < static_cast<unsigned>( m_params.minPointsPerSegment ) )
	{
		m_lastError = "Insufficient number of points";
		return false;
	}
	
	try
	{
#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
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
#else
		// Fallback implementation - use original algorithm
		return extractFallback(cloud, centerlines);
#endif
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Exception: %1" ).arg( e.what() );
		return false;
	}
}

// Fallback implementation when Open3D is not available
bool PipeCenterlineExtractor::extractFallback(ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines)
{
	ccPointCloud* processedCloud = nullptr;
	
	try
	{
		// Step 1: Preprocess point cloud
		if ( !preprocess( cloud, processedCloud ) )
		{
			if ( processedCloud )
			{
				delete processedCloud;
			}
			return false;
		}
		
		// Step 2: Estimate normals
		if ( !estimateNormals( processedCloud ) )
		{
			delete processedCloud;
			return false;
		}
		
		// Step 3: Extract pipe points
		std::vector<int> pipePointIndices;
		if ( !extractPipePoints( processedCloud, pipePointIndices ) )
		{
			delete processedCloud;
			return false;
		}
		
		if ( pipePointIndices.empty() )
		{
			m_lastError = "No pipe points detected";
			delete processedCloud;
			return false;
		}
		
		// Create subset cloud with pipe points only
		ccPointCloud* pipeCloud = new ccPointCloud( "pipe_points" );
		for ( int index : pipePointIndices )
		{
			pipeCloud->addPoint( *processedCloud->getPoint( index ) );
		}
		
		// Step 4: Compute skeleton
		std::vector<std::vector<CCVector3>> skeletonPaths;
		if ( !computeSkeleton( pipeCloud, skeletonPaths ) )
		{
			delete processedCloud;
			delete pipeCloud;
			return false;
		}
		
		// Step 5: Handle branches if enabled
		std::vector<std::vector<CCVector3>> finalPaths;
		if ( m_params.useBranchDetection )
		{
			if ( !detectBranches( skeletonPaths, finalPaths ) )
			{
				finalPaths = skeletonPaths;
			}
		}
		else
		{
			finalPaths = skeletonPaths;
		}
		
		// Step 6: Smooth paths
		if ( !smoothPaths( finalPaths ) )
		{
			delete processedCloud;
			delete pipeCloud;
			return false;
		}
		
		// Step 7: Create polylines
		if ( !createPolylines( finalPaths, centerlines ) )
		{
			delete processedCloud;
			delete pipeCloud;
			return false;
		}
		
		// Cleanup
		delete processedCloud;
		delete pipeCloud;
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Exception: %1" ).arg( e.what() );
		if ( processedCloud )
		{
			delete processedCloud;
		}
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

ccPointCloud* PipeCenterlineExtractor::open3DToCC(const std::shared_ptr<open3d::geometry::PointCloud>& o3dCloud, const QString& name)
{
	if (!o3dCloud || o3dCloud->points_.empty())
	{
		return nullptr;
	}
	
	ccPointCloud* ccCloud = new ccPointCloud(name);
	
	// Convert points
	for (const auto& point : o3dCloud->points_)
	{
		ccCloud->addPoint(eigenToCC(point));
	}
	
	// Convert colors if available
	if (!o3dCloud->colors_.empty() && o3dCloud->colors_.size() == o3dCloud->points_.size())
	{
		ccCloud->resizeTheRGBTable(false);
		for (unsigned i = 0; i < o3dCloud->colors_.size(); ++i)
		{
			const Eigen::Vector3d& color = o3dCloud->colors_[i];
			ccColor::Rgb c(static_cast<ColorCompType>(color.x() * 255),
						  static_cast<ColorCompType>(color.y() * 255),
						  static_cast<ColorCompType>(color.z() * 255));
			ccCloud->addRGBColor(c);
		}
	}
	
	return ccCloud;
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
		int longest_axis;
		extent.maxCoeff(&longest_axis);
		
		Eigen::Vector3d start = centroid;
		Eigen::Vector3d end = centroid;
		start[longest_axis] = min_pt[longest_axis];
		end[longest_axis] = max_pt[longest_axis];
		
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

	
	try
	{
		// Build KD-tree for efficient neighbor search
		open3d::geometry::KDTreeFlann kdtree(*cloud);
		
		// Compute distance transform (approximate medial axis)
		std::vector<double> distances(cloud->points_.size(), 0.0);
		double search_radius = m_params.radiusEstimate * 2.0;
		
		for (size_t i = 0; i < cloud->points_.size(); ++i)
		{
			std::vector<int> indices;
			std::vector<double> dists;
			kdtree.SearchRadius(cloud->points_[i], search_radius, indices, dists);
			
			if (!indices.empty())
			{
				// Use average distance as approximate distance transform
				double avg_dist = 0.0;
				for (double d : dists)
				{
					avg_dist += std::sqrt(d);
				}
				distances[i] = avg_dist / dists.size();
			}
		}
		
		// Extract local maxima (skeleton points)
		std::vector<size_t> skeleton_indices;
		double threshold = 0.6; // Relative threshold for skeleton detection
		
		for (size_t i = 0; i < cloud->points_.size(); ++i)
		{
			if (distances[i] < threshold * m_params.radiusEstimate)
			{
				continue;
			}
			
			// Check if local maximum
			std::vector<int> neighbors;
			std::vector<double> neighbor_dists;
			kdtree.SearchRadius(cloud->points_[i], search_radius * 0.5, neighbors, neighbor_dists);
			
			bool is_local_max = true;
			for (size_t j = 1; j < neighbors.size(); ++j) // Skip first (itself)
			{
				if (distances[neighbors[j]] > distances[i])
				{
					is_local_max = false;
					break;
				}
			}
			
			if (is_local_max)
			{
				skeleton_indices.push_back(i);
			}
		}
		
		// Connect skeleton points into continuous paths
		if (skeleton_indices.size() >= 2)
		{
		
		return centerlines;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Point-based centerline computation failed: %1").arg(e.what());
		return centerlines;
	}
}

	
	// Build KD-tree
	open3d::geometry::KDTreeFlann kdtree(*cloud);
	
	// Track used points
	std::vector<bool> used(skeleton_indices.size(), false);
	double connection_threshold = m_params.radiusEstimate * 1.5;
	
	for (size_t i = 0; i < skeleton_indices.size(); ++i)
	{
		if (used[i]) continue;
		
		std::vector<Eigen::Vector3d> path;
		path.push_back(cloud->points_[skeleton_indices[i]]);
		used[i] = true;
		
		// Grow path in both directions
		bool extended = true;
		while (extended)
		{
			extended = false;
			
			// Try to extend at both ends
			for (int end = 0; end < 2; ++end)
			{
				Eigen::Vector3d current_point = end == 0 ? path.front() : path.back();
				
				// Find nearest skeleton point
				int best_idx = -1;
				double best_dist = std::numeric_limits<double>::max();
				
				for (size_t j = 0; j < skeleton_indices.size(); ++j)
				{
					if (used[j]) continue;
					
					double dist = (cloud->points_[skeleton_indices[j]] - current_point).norm();
					if (dist < best_dist && dist < connection_threshold)
					{
						best_dist = dist;
						best_idx = j;
					}
				}
				
				if (best_idx >= 0)
				{
					if (end == 0)
					{
						path.insert(path.begin(), cloud->points_[skeleton_indices[best_idx]]);
					}
					else
					{
						path.push_back(cloud->points_[skeleton_indices[best_idx]]);
					}
					used[best_idx] = true;
					extended = true;
				}
			}
		}
		
		if (path.size() >= 2)
		{
			centerlines.push_back(path);
		}
	}
	
	return centerlines;
}

	
	// Simplified medial axis computation using distance from boundary
	// This is a basic implementation - for production, consider more sophisticated algorithms
	
	try
	{
		// Compute vertex normals if not available
		auto mesh_with_normals = std::make_shared<open3d::geometry::TriangleMesh>(*mesh);
		if (!mesh_with_normals->HasVertexNormals())
		{
			mesh_with_normals->ComputeVertexNormals();
		}
		
		// Identify potential medial axis points based on distance from boundary
		// and local geometric properties
		std::vector<size_t> medial_points;
		
		for (size_t i = 0; i < mesh_with_normals->vertices_.size(); ++i)
		{
			const Eigen::Vector3d& vertex = mesh_with_normals->vertices_[i];
			const Eigen::Vector3d& normal = mesh_with_normals->vertex_normals_[i];
			
			// Simple heuristic: points with high distance to boundary
			// and consistent normal directions are potential medial axis points
			
			// This is a simplified approach - in practice, you'd want more
			// sophisticated medial axis transform algorithms
			
			// For now, return empty and fall back to point-based approach
			break;
		}
		
		return medial_axis;
	}
	catch (const std::exception& e)
	{
		m_lastError = QString("Medial axis computation failed: %1").arg(e.what());
		return medial_axis;
	}
}

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

	
	// Calculate direction vector at given index
	if (index == 0)
	{
		return (line[1] - line[0]).normalized();
	}
	else if (index == line.size() - 1)
	{
		return (line[index] - line[index - 1]).normalized();
	}
	else
	{
		// Average of forward and backward directions
		Eigen::Vector3d forward = (line[index + 1] - line[index]).normalized();
		Eigen::Vector3d backward = (line[index] - line[index - 1]).normalized();
		return ((forward + backward) / 2.0).normalized();
	}
}

std::vector<std::vector<Eigen::Vector3d>> PipeCenterlineExtractor::smoothPathsOpen3D(const std::vector<std::vector<Eigen::Vector3d>>& paths)
{
	// For now, just return the input paths without smoothing
	// Smoothing can be implemented later if needed
	return paths;
}

bool PipeCenterlineExtractor::createPolylinesFromPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths,
													  std::vector<ccPolyline*>& polylines)
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

#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
// Open3D-specific implementations

bool PipeCenterlineExtractor::preprocess( ccPointCloud* cloud, ccPointCloud*& processedCloud )
{
	if ( !cloud )
	{
		m_lastError = "Invalid input cloud for preprocessing";
		return false;
	}
	
	try
	{
		// Create a copy of the original cloud
		processedCloud = new ccPointCloud( cloud->getName() + "_processed" );
		*processedCloud = *cloud;
		
		// Simple voxel grid filtering (uniform sampling)
		if ( m_params.voxelSize > 0.0 )
		{
			// Create a temporary scalar field for spatial indexing
			int sfIdx = processedCloud->addScalarField( "temp_index" );
			if ( sfIdx < 0 )
			{
				delete processedCloud;
				m_lastError = "Failed to create temporary scalar field";
				return false;
			}
			
			CCCoreLib::ScalarField* sf = processedCloud->getScalarField( sfIdx );
			
			// Simple uniform sampling based on voxel grid
			std::vector<bool> keepPoint( processedCloud->size(), true );
			std::map<std::tuple<int, int, int>, bool> voxelGrid;
			
			for ( unsigned i = 0; i < processedCloud->size(); ++i )
			{
				const CCVector3* P = processedCloud->getPoint( i );
				
				int vx = static_cast<int>( P->x / m_params.voxelSize );
				int vy = static_cast<int>( P->y / m_params.voxelSize );
				int vz = static_cast<int>( P->z / m_params.voxelSize );
				
				auto key = std::make_tuple( vx, vy, vz );
				if ( voxelGrid.find( key ) != voxelGrid.end() )
				{
					keepPoint[i] = false;
				}
				else
				{
					voxelGrid[key] = true;
				}
			}
			
			// Create new cloud with filtered points
			ccPointCloud* filteredCloud = new ccPointCloud( processedCloud->getName() + "_filtered" );
			for ( unsigned i = 0; i < processedCloud->size(); ++i )
			{
				if ( keepPoint[i] )
				{
					filteredCloud->addPoint( *processedCloud->getPoint( i ) );
				}
			}
			
			delete processedCloud;
			processedCloud = filteredCloud;
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		if ( processedCloud )
		{
			delete processedCloud;
		}
		m_lastError = QString( "Preprocessing failed: %1" ).arg( e.what() );
		return false;
	}
}

bool PipeCenterlineExtractor::estimateNormals( ccPointCloud* cloud )
{
	if ( !cloud )
	{
		m_lastError = "Invalid cloud for normal estimation";
		return false;
	}
	
	try
	{
		// Reserve space for normals
		m_normals.resize( cloud->size() );
		
		// Use a simple local plane fitting approach
		double searchRadius = m_params.radiusEstimate * 2.0;
		
		for ( unsigned i = 0; i < cloud->size(); ++i )
		{
			const CCVector3* P = cloud->getPoint( i );
			
			// Find neighbors
			std::vector<unsigned> neighbors = findNeighbors( cloud, i, searchRadius );
			
			if ( neighbors.size() < 3 )
			{
				m_normals[i] = CCVector3( 0, 0, 1 ); // Default normal
				continue;
			}
			
			// Compute centroid
			CCVector3 centroid( 0, 0, 0 );
			for ( unsigned neighborIdx : neighbors )
			{
				centroid += *cloud->getPoint( neighborIdx );
			}
			centroid /= static_cast<double>( neighbors.size() );
			
			// Compute covariance matrix
			double cov[9] = { 0 };
			for ( unsigned neighborIdx : neighbors )
			{
				CCVector3 diff = *cloud->getPoint( neighborIdx ) - centroid;
				cov[0] += diff.x * diff.x;
				cov[1] += diff.x * diff.y;
				cov[2] += diff.x * diff.z;
				cov[4] += diff.y * diff.y;
				cov[5] += diff.y * diff.z;
				cov[8] += diff.z * diff.z;
			}
			cov[3] = cov[1];
			cov[6] = cov[2];
			cov[7] = cov[5];
			
			// Simple eigenvalue decomposition to find smallest eigenvector
			// For now, use a simplified approach
			CCVector3 normal( 0, 0, 1 );
			
			// Use cross product of two vectors to get approximate normal
			if ( neighbors.size() >= 3 )
			{
				CCVector3 v1 = *cloud->getPoint( neighbors[1] ) - *cloud->getPoint( neighbors[0] );
				CCVector3 v2 = *cloud->getPoint( neighbors[2] ) - *cloud->getPoint( neighbors[0] );
				normal = v1.cross( v2 );
				normal.normalize();
			}
			
			m_normals[i] = normal;
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Normal estimation failed: %1" ).arg( e.what() );
		return false;
	}
}

bool PipeCenterlineExtractor::extractPipePoints( ccPointCloud* cloud, std::vector<int>& pipePointIndices )
{
	if ( !cloud )
	{
		m_lastError = "Invalid cloud for pipe point extraction";
		return false;
	}
	
	try
	{
		m_isPipePoint.resize( cloud->size(), false );
		pipePointIndices.clear();
		
		for ( unsigned i = 0; i < cloud->size(); ++i )
		{
			if ( isPipePoint( cloud, i ) )
			{
				m_isPipePoint[i] = true;
				pipePointIndices.push_back( i );
			}
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Pipe point extraction failed: %1" ).arg( e.what() );
		return false;
	}
}

bool PipeCenterlineExtractor::computeSkeleton( ccPointCloud* cloud, std::vector<std::vector<CCVector3>>& skeletonPaths )
{
	if ( !cloud )
	{
		m_lastError = "Invalid cloud for skeleton computation";
		return false;
	}
	
	try
	{
		skeletonPaths.clear();
		
		if ( cloud->size() < 10 )
		{
			m_lastError = "Insufficient points for skeleton computation";
			return false;
		}
		
		// Simple medial axis extraction using distance transform
		// For each point, find the maximum distance to boundary
		std::vector<double> distances( cloud->size(), 0.0 );
		
		// Compute approximate distances (simplified approach)
		double searchRadius = m_params.radiusEstimate * 3.0;
		
		for ( unsigned i = 0; i < cloud->size(); ++i )
		{
			const CCVector3* P = cloud->getPoint( i );
			
			// Find neighbors and compute average distance
			std::vector<unsigned> neighbors = findNeighbors( cloud, i, searchRadius );
			
			if ( neighbors.size() < 5 )
			{
				distances[i] = 0.0;
				continue;
			}
			
			double avgDistance = 0.0;
			for ( unsigned neighborIdx : neighbors )
			{
				CCVector3 diff = *cloud->getPoint( neighborIdx ) - *P;
				avgDistance += diff.norm();
			}
			avgDistance /= neighbors.size();
			
			distances[i] = avgDistance;
		}
		
		// Extract skeleton points (local maxima of distance)
		std::vector<unsigned> skeletonPointIndices;
		double threshold = 0.7; // Relative threshold
		
		for ( unsigned i = 0; i < cloud->size(); ++i )
		{
			if ( distances[i] < threshold * m_params.radiusEstimate )
			{
				continue;
			}
			
			// Check if local maximum
			std::vector<unsigned> neighbors = findNeighbors( cloud, i, searchRadius * 0.5 );
			bool isLocalMax = true;
			
			for ( unsigned neighborIdx : neighbors )
			{
				if ( distances[neighborIdx] > distances[i] )
				{
					isLocalMax = false;
					break;
				}
			}
			
			if ( isLocalMax )
			{
				skeletonPointIndices.push_back( i );
			}
		}
		
		// Connect skeleton points into paths
		if ( skeletonPointIndices.size() < 2 )
		{
			m_lastError = "Insufficient skeleton points";
			return false;
		}
		
		// Simple path connection based on proximity
		std::vector<bool> used( skeletonPointIndices.size(), false );
		
		for ( size_t i = 0; i < skeletonPointIndices.size(); ++i )
		{
			if ( used[i] )
			{
				continue;
			}
			
			std::vector<CCVector3> path;
			path.push_back( *cloud->getPoint( skeletonPointIndices[i] ) );
			used[i] = true;
			
			// Find connected points
			bool foundConnection = true;
			while ( foundConnection )
			{
				foundConnection = false;
				
				for ( size_t j = 0; j < skeletonPointIndices.size(); ++j )
				{
					if ( used[j] )
					{
						continue;
					}
					
					CCVector3 diff = *cloud->getPoint( skeletonPointIndices[j] ) - path.back();
					if ( diff.norm() < searchRadius * 0.8 )
					{
						path.push_back( *cloud->getPoint( skeletonPointIndices[j] ) );
						used[j] = true;
						foundConnection = true;
						break;
					}
				}
			}
			
			if ( path.size() >= 2 )
			{
				skeletonPaths.push_back( path );
			}
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Skeleton computation failed: %1" ).arg( e.what() );
		return false;
	}
}

bool PipeCenterlineExtractor::detectBranches( const std::vector<std::vector<CCVector3>>& skeletonPaths,
											  std::vector<std::vector<CCVector3>>& branchedPaths )
{
	// For now, just copy the input paths
	// TODO: Implement proper branch detection and handling
	branchedPaths = skeletonPaths;
	return true;
}

bool PipeCenterlineExtractor::smoothPaths( std::vector<std::vector<CCVector3>>& paths )
{
	try
	{
		for ( auto& path : paths )
		{
			if ( path.size() < 3 )
			{
				continue;
			}
			
			// Simple moving average smoothing
			std::vector<CCVector3> smoothedPath;
			smoothedPath.reserve( path.size() );
			
			for ( size_t i = 0; i < path.size(); ++i )
			{
				CCVector3 avg( 0, 0, 0 );
				int count = 0;
				
				// Average with neighbors
				int start = std::max( 0, static_cast<int>( i ) - 2 );
				int end = std::min( static_cast<int>( path.size() ) - 1, static_cast<int>( i ) + 2 );
				
				for ( int j = start; j <= end; ++j )
				{
					avg += path[j];
					count++;
				}
				
				if ( count > 0 )
				{
					avg /= count;
					smoothedPath.push_back( avg );
				}
			}
			
			path = smoothedPath;
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Path smoothing failed: %1" ).arg( e.what() );
		return false;
	}
}

bool PipeCenterlineExtractor::createPolylines( const std::vector<std::vector<CCVector3>>& paths,
											   std::vector<ccPolyline*>& polylines )
{
	try
	{
		polylines.clear();
		
		for ( size_t i = 0; i < paths.size(); ++i )
		{
			const auto& path = paths[i];
			if ( path.size() < 2 )
			{
				continue;
			}
			
			// Create point cloud for polyline vertices
			ccPointCloud* vertices = new ccPointCloud( QString( "vertices_%1" ).arg( i ) );
			for ( const auto& point : path )
			{
				vertices->addPoint( point );
			}
			
			// Create polyline
			ccPolyline* polyline = new ccPolyline( vertices );
			polyline->setName( QString( "centerline_%1" ).arg( i ) );
			polyline->addPointIndex( 0, static_cast<unsigned>( path.size() ) );
			polyline->setClosed( false );
			
			polylines.push_back( polyline );
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Polyline creation failed: %1" ).arg( e.what() );
		return false;
	}
}

double PipeCenterlineExtractor::computeCurvature( ccPointCloud* cloud, unsigned pointIndex )
{
	if ( !cloud || pointIndex >= cloud->size() )
	{
		return 0.0;
	}
	
	// Simple curvature estimation based on local point distribution
	std::vector<unsigned> neighbors = findNeighbors( cloud, pointIndex, m_params.radiusEstimate );
	
	if ( neighbors.size() < 5 )
	{
		return 0.0;
	}
	
	const CCVector3* center = cloud->getPoint( pointIndex );
	
	// Compute local covariance
	double cov[9] = { 0 };
	CCVector3 centroid( 0, 0, 0 );
	
	for ( unsigned neighborIdx : neighbors )
	{
		centroid += *cloud->getPoint( neighborIdx );
	}
	centroid /= neighbors.size();
	
	for ( unsigned neighborIdx : neighbors )
	{
		CCVector3 diff = *cloud->getPoint( neighborIdx ) - centroid;
		cov[0] += diff.x * diff.x;
		cov[1] += diff.x * diff.y;
		cov[2] += diff.x * diff.z;
		cov[4] += diff.y * diff.y;
		cov[5] += diff.y * diff.z;
		cov[8] += diff.z * diff.z;
	}
	cov[3] = cov[1];
	cov[6] = cov[2];
	cov[7] = cov[5];
	
	// Simple curvature measure: ratio of smallest to largest eigenvalue
	// For now, use a simplified approach based on trace and determinant
	double trace = cov[0] + cov[4] + cov[8];
	
	// Simplified curvature estimation
	double curvature = 0.0;
	if ( trace > 0 )
	{
		curvature = std::sqrt( 1.0 / ( trace / neighbors.size() ) );
	}
	
	return curvature;
}

bool PipeCenterlineExtractor::isPipePoint( ccPointCloud* cloud, unsigned pointIndex )
{
	if ( !cloud || pointIndex >= cloud->size() )
	{
		return false;
	}
	
	// Check if point has characteristics of a pipe surface point
	double curvature = computeCurvature( cloud, pointIndex );
	
	// Pipe points should have relatively low curvature
	if ( curvature > m_params.curvatureThreshold )
	{
		return false;
	}
	
	// Check local geometry
	std::vector<unsigned> neighbors = findNeighbors( cloud, pointIndex, m_params.radiusEstimate * 2.0 );
	
	if ( neighbors.size() < 10 )
	{
		return false;
	}
	
	// Check if points form a cylindrical pattern
	// For now, use a simplified heuristic
	const CCVector3* center = cloud->getPoint( pointIndex );
	
	// Compute average distance to neighbors
	double avgDistance = 0.0;
	for ( unsigned neighborIdx : neighbors )
	{
		CCVector3 diff = *cloud->getPoint( neighborIdx ) - *center;
		avgDistance += diff.norm();
	}
	avgDistance /= neighbors.size();
	
	// Check if distances are relatively uniform (indicating cylindrical shape)
	double variance = 0.0;
	for ( unsigned neighborIdx : neighbors )
	{
		CCVector3 diff = *cloud->getPoint( neighborIdx ) - *center;
		double distance = diff.norm();
		variance += ( distance - avgDistance ) * ( distance - avgDistance );
	}
	variance /= neighbors.size();
	
	// Low variance indicates more uniform distances (cylindrical pattern)
	return variance < ( m_params.radiusEstimate * m_params.radiusEstimate * 0.5 );
}

std::vector<unsigned> PipeCenterlineExtractor::findNeighbors( ccPointCloud* cloud, unsigned pointIndex, double radius )
{
	std::vector<unsigned> neighbors;
	
	if ( !cloud || pointIndex >= cloud->size() )
	{
		return neighbors;
	}
	
	const CCVector3* center = cloud->getPoint( pointIndex );
	
	for ( unsigned i = 0; i < cloud->size(); ++i )
	{
		if ( i == pointIndex )
		{
			continue;
		}
		
		CCVector3 diff = *cloud->getPoint( i ) - *center;
		if ( diff.norm() <= radius )
		{
			neighbors.push_back( i );
		}
	}
	
	return neighbors;
}

#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
// Open3D-specific implementations

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

ccPointCloud* PipeCenterlineExtractor::open3DToCC(const std::shared_ptr<open3d::geometry::PointCloud>& o3dCloud, const QString& name)
{
	if (!o3dCloud || o3dCloud->points_.empty())
	{
		return nullptr;
	}
	
	ccPointCloud* ccCloud = new ccPointCloud(name);
	
	// Convert points
	for (const auto& point : o3dCloud->points_)
	{
		ccCloud->addPoint(eigenToCC(point));
	}
	
	// Convert colors if available
	if (!o3dCloud->colors_.empty() && o3dCloud->colors_.size() == o3dCloud->points_.size())
	{
		ccCloud->resizeTheRGBTable(false);
		for (unsigned i = 0; i < o3dCloud->colors_.size(); ++i)
		{
			const Eigen::Vector3d& color = o3dCloud->colors_[i];
			ccColor::Rgb c(static_cast<ColorCompType>(color.x() * 255),
						  static_cast<ColorCompType>(color.y() * 255),
						  static_cast<ColorCompType>(color.z() * 255));
			ccCloud->addRGBColor(c);
		}
	}
	
	return ccCloud;
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
		int longest_axis;
		extent.maxCoeff(&longest_axis);
		
		Eigen::Vector3d start = centroid;
		Eigen::Vector3d end = centroid;
		start[longest_axis] = min_pt[longest_axis];
		end[longest_axis] = max_pt[longest_axis];
		
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
													  std::vector<ccPolyline*>& polylines)
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

#endif // USE_OPEN3D_WITH_PIPECENTERLINE