#pragma once

// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
// #                                                                        #
// ##########################################################################

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <Neighbourhood.h>

// System
#include <vector>
#include <memory>

//! Cross section feature extraction algorithm
class CrossSectionExtractor
{
public:
	//! Parameters for feature extraction
	struct Parameters
	{
		double planeResolution;      //!< Resolution for plane fitting
		double featureThreshold;     //!< Threshold for feature detection
		int minPointsPerFeature;     //!< Minimum points per feature
		double searchRadius;         //!< Search radius for neighbors
		bool useAdvancedFeatures;    //!< Enable advanced feature detection
		
		//! Default constructor with default values
		Parameters()
			: planeResolution(0.01)
			, featureThreshold(0.1)
			, minPointsPerFeature(10)
			, searchRadius(0.05)
			, useAdvancedFeatures(false)
		{}
	};

	//! Constructor
	explicit CrossSectionExtractor( const Parameters& params = Parameters() );
	
	//! Destructor
	~CrossSectionExtractor() = default;
	
	//! Extract features from cross section point cloud
	bool extract( ccPointCloud* cloud, std::vector<ccPolyline*>& features );
	
	//! Get the last error message
	QString getLastError() const { return m_lastError; }
	
	//! Set parameters
	void setParameters( const Parameters& params ) { m_params = params; }
	
	//! Get parameters
	const Parameters& getParameters() const { return m_params; }

private:
	//! Extract cross section plane
	bool extractCrossSectionPlane( ccPointCloud* cloud );
	
	//! Detect feature points
	bool detectFeaturePoints( ccPointCloud* cloud, std::vector<int>& featureIndices );
	
	//! Create feature polylines
	bool createFeaturePolylines( const std::vector<int>& featureIndices, 
								 std::vector<ccPolyline*>& features );
	
	//! Parameters
	Parameters m_params;
	
	//! Last error message
	QString m_lastError;
};