#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>

// Conditionally include Open3D
#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
#include <open3d/Open3D.h>
#endif

// System
#include <memory>
#include <vector>

//! Pipe centerline extraction algorithm using Open3D
class PipeCenterlineExtractor
{
  public:
	//! Parameters for centerline extraction
	struct Parameters
	{
		double radiusEstimate;       //!< Estimated pipe radius
		double voxelSize;            //!< Voxel size for downsampling
		int    minPointsPerSegment;  //!< Minimum points per segment
		double curvatureThreshold;   //!< Curvature threshold for pipe detection
		double distanceThreshold;    //!< Distance threshold for point clustering
		bool   useBranchDetection;   //!< Enable branch detection
		double branchAngleThreshold; //!< Branch angle threshold in degrees
#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
		bool   useRANSAC;               //!< Use RANSAC for cylinder fitting
		double ransacDistanceThreshold; //!< RANSAC distance threshold
		int    ransacMaxIterations;     //!< RANSAC maximum iterations
		bool   useMLSR;                 //!< Use Moving Least Squares smoothing
		double mlsrSearchRadius;        //!< MLSR search radius
#endif

		//! Default constructor with default values
		Parameters()
		    : radiusEstimate(0.1)
		    , voxelSize(0.02)
		    , minPointsPerSegment(50)
		    , curvatureThreshold(0.1)
		    , distanceThreshold(0.05)
		    , useBranchDetection(true)
		    , branchAngleThreshold(30.0)
#ifdef USE_OPEN3D_WITH_PIPECENTERLINE
		    , useRANSAC(true)
		    , ransacDistanceThreshold(0.01)
		    , ransacMaxIterations(1000)
		    , useMLSR(true)
		    , mlsrSearchRadius(0.05)
#endif
		{
		}
	};

	//! Constructor
	explicit PipeCenterlineExtractor(const Parameters& params = Parameters());

	//! Destructor
	~PipeCenterlineExtractor() = default;

	//! Extract centerline from point cloud
	bool extract(ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines);

	//! Get the last error message
	QString getLastError() const
	{
		return m_lastError;
	}

	//! Set parameters
	void setParameters(const Parameters& params)
	{
		m_params = params;
	}

	//! Get parameters
	const Parameters& getParameters() const
	{
		return m_params;
	}

  private:
	//! Convert CloudCompare point cloud to Open3D point cloud
	std::shared_ptr<open3d::geometry::PointCloud> ccToOpen3D(ccPointCloud* cloud);

	//! Convert Open3D point cloud to CloudCompare point cloud
	ccPointCloud* open3DToCC(const std::shared_ptr<open3d::geometry::PointCloud>& o3dCloud, const QString& name);

	//! Preprocess point cloud using Open3D
	std::shared_ptr<open3d::geometry::PointCloud> preprocessOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud);

	//! Extract pipe points using Open3D geometry features
	std::shared_ptr<open3d::geometry::PointCloud> extractPipePointsOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud);

	//! Compute centerline using Open3D skeletonization
	std::vector<std::vector<Eigen::Vector3d>> computeCenterlineOpen3D(const std::shared_ptr<open3d::geometry::PointCloud>& cloud);

	//! Detect and handle branches using Open3D
	std::vector<std::vector<Eigen::Vector3d>> detectBranchesOpen3D(const std::vector<std::vector<Eigen::Vector3d>>& centerlines);

	//! Smooth centerline paths
	std::vector<std::vector<Eigen::Vector3d>> smoothPathsOpen3D(const std::vector<std::vector<Eigen::Vector3d>>& paths);

	//! Create CloudCompare polylines from Open3D paths
	bool createPolylinesFromPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths,
	                              std::vector<ccPolyline*>&                        polylines);

	//! Convert Eigen vector to CCVector3
	CCVector3 eigenToCC(const Eigen::Vector3d& eigenVec);

	//! Convert CCVector3 to Eigen vector
	Eigen::Vector3d ccToEigen(const CCVector3& ccVec);

	//! Get line direction at specific index
	Eigen::Vector3d getLineDirection(const std::vector<Eigen::Vector3d>& line, size_t index);
#endif

	// Fallback implementation for when Open3D is not available
	bool extractFallback(ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines);

	// Original fallback methods
	bool                  preprocess(ccPointCloud* cloud, ccPointCloud*& processedCloud);
	bool                  estimateNormals(ccPointCloud* cloud);
	bool                  extractPipePoints(ccPointCloud* cloud, std::vector<int>& pipePointIndices);
	bool                  computeSkeleton(ccPointCloud* cloud, std::vector<std::vector<CCVector3>>& skeletonPaths);
	bool                  detectBranches(const std::vector<std::vector<CCVector3>>& skeletonPaths,
	                                     std::vector<std::vector<CCVector3>>&       branchedPaths);
	bool                  smoothPaths(std::vector<std::vector<CCVector3>>& paths);
	bool                  createPolylines(const std::vector<std::vector<CCVector3>>& paths,
	                                      std::vector<ccPolyline*>&                  polylines);
	double                computeCurvature(ccPointCloud* cloud, unsigned pointIndex);
	bool                  isPipePoint(ccPointCloud* cloud, unsigned pointIndex);
	std::vector<unsigned> findNeighbors(ccPointCloud* cloud, unsigned pointIndex, double radius);

	//! Parameters
	Parameters m_params;

	//! Last error message
	QString m_lastError;

	// Processing data for fallback implementation
	std::vector<CCVector3> m_normals;
	std::vector<bool>      m_isPipePoint;
};