#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <Neighbourhood.h>

// System
#include <vector>
#include <memory>

//! Pipe centerline extraction algorithm
class PipeCenterlineExtractor
{
public:
	//! Parameters for centerline extraction
	struct Parameters
	{
		double radiusEstimate;      //!< Estimated pipe radius
		double voxelSize;          //!< Voxel size for downsampling
		int minPointsPerSegment;     //!< Minimum points per segment
		double curvatureThreshold;  //!< Curvature threshold for pipe detection
		double distanceThreshold;  //!< Distance threshold for point clustering
		bool useBranchDetection;   //!< Enable branch detection
		double branchAngleThreshold; //!< Branch angle threshold in degrees
		
		//! Default constructor with default values
		Parameters()
			: radiusEstimate(0.1)
			, voxelSize(0.02)
			, minPointsPerSegment(50)
			, curvatureThreshold(0.1)
			, distanceThreshold(0.05)
			, useBranchDetection(true)
			, branchAngleThreshold(30.0)
		{}
	};

	//! Constructor
	explicit PipeCenterlineExtractor( const Parameters& params = Parameters() );
	
	//! Destructor
	~PipeCenterlineExtractor() = default;
	
	//! Extract centerline from point cloud
	bool extract( ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines );
	
	//! Get the last error message
	QString getLastError() const { return m_lastError; }
	
	//! Set parameters
	void setParameters( const Parameters& params ) { m_params = params; }
	
	//! Get parameters
	const Parameters& getParameters() const { return m_params; }

private:
	//! Preprocess point cloud
	bool preprocess( ccPointCloud* cloud, ccPointCloud*& processedCloud );
	
	//! Estimate normals
	bool estimateNormals( ccPointCloud* cloud );
	
	//! Extract pipe points based on geometric features
	bool extractPipePoints( ccPointCloud* cloud, std::vector<int>& pipePointIndices );
	
	//! Compute skeleton using thinning algorithm
	bool computeSkeleton( ccPointCloud* cloud, std::vector<std::vector<CCVector3>>& skeletonPaths );
	
	//! Detect and handle branches
	bool detectBranches( const std::vector<std::vector<CCVector3>>& skeletonPaths,
						 std::vector<std::vector<CCVector3>>& branchedPaths );
	
	//! Smooth skeleton paths
	bool smoothPaths( std::vector<std::vector<CCVector3>>& paths );
	
	//! Create polylines from paths
	bool createPolylines( const std::vector<std::vector<CCVector3>>& paths,
						  std::vector<ccPolyline*>& polylines );
	
	//! Compute local curvature
	double computeCurvature( ccPointCloud* cloud, unsigned pointIndex );
	
	//! Check if point is on pipe surface
	bool isPipePoint( ccPointCloud* cloud, unsigned pointIndex );
	
	//! Find neighboring points
	std::vector<unsigned> findNeighbors( ccPointCloud* cloud, unsigned pointIndex, double radius );
	
	//! Parameters
	Parameters m_params;
	
	//! Last error message
	QString m_lastError;
	
	//! Processing data
	std::vector<CCVector3> m_normals;
	std::vector<bool> m_isPipePoint;
};