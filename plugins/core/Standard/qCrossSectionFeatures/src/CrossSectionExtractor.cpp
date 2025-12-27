// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
// #                                                                        #
// ##########################################################################

#include "../include/CrossSectionExtractor.h"

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>
#include <Neighbourhood.h>
#include <DistanceComputationTools.h>
#include <ScalarField.h>

// System
#include <queue>
#include <algorithm>
#include <cmath>

CrossSectionExtractor::CrossSectionExtractor( const Parameters& params )
	: m_params( params )
{
}

bool CrossSectionExtractor::extract( ccPointCloud* cloud, std::vector<ccPolyline*>& features )
{
	if ( !cloud )
	{
		m_lastError = "Invalid input point cloud";
		return false;
	}
	
	if ( cloud->size() < static_cast<unsigned>( m_params.minPointsPerFeature ) )
	{
		m_lastError = "Insufficient number of points";
		return false;
	}
	
	try
	{
		// Step 1: Extract cross section plane
		if ( !extractCrossSectionPlane( cloud ) )
		{
			return false;
		}
		
		// Step 2: Detect feature points
		std::vector<int> featureIndices;
		if ( !detectFeaturePoints( cloud, featureIndices ) )
		{
			return false;
		}
		
		if ( featureIndices.empty() )
		{
			m_lastError = "No features detected";
			return false;
		}
		
		// Step 3: Create feature polylines
		if ( !createFeaturePolylines( featureIndices, features ) )
		{
			return false;
		}
		
		return true;
	}
	catch ( const std::exception& e )
	{
		m_lastError = QString( "Exception: %1" ).arg( e.what() );
		return false;
	}
}

bool CrossSectionExtractor::extractCrossSectionPlane( ccPointCloud* cloud )
{
	if ( !cloud )
	{
		m_lastError = "Invalid cloud for plane extraction";
		return false;
	}
	
	// Basic implementation: assume the cross section is roughly planar
	// For now, just return true - detailed implementation can be added later
	return true;
}

bool CrossSectionExtractor::detectFeaturePoints( ccPointCloud* cloud, std::vector<int>& featureIndices )
{
	if ( !cloud )
	{
		m_lastError = "Invalid cloud for feature detection";
		return false;
	}
	
	featureIndices.clear();
	
	// Basic implementation: detect points with high curvature or significant geometric features
	// For now, select some sample points as features
	
	unsigned step = std::max( 1u, cloud->size() / 20 );  // Sample ~20 points
	for ( unsigned i = 0; i < cloud->size(); i += step )
	{
		featureIndices.push_back( i );
	}
	
	// Ensure we have at least minPointsPerFeature
	if ( featureIndices.size() < static_cast<size_t>( m_params.minPointsPerFeature ) )
	{
		// Add more points if needed
		for ( unsigned i = 0; i < cloud->size() && featureIndices.size() < static_cast<size_t>( m_params.minPointsPerFeature ); ++i )
		{
			if ( std::find( featureIndices.begin(), featureIndices.end(), i ) == featureIndices.end() )
			{
				featureIndices.push_back( i );
			}
		}
	}
	
	return true;
}

bool CrossSectionExtractor::createFeaturePolylines( const std::vector<int>& featureIndices, 
													 std::vector<ccPolyline*>& features )
{
	if ( featureIndices.empty() )
	{
		m_lastError = "No feature indices provided";
		return false;
	}
	
	// Basic implementation: create a simple polyline connecting all feature points
	// For now, create one polyline with all points
	
	ccPointCloud* vertices = new ccPointCloud( "feature_vertices" );
	std::vector<CCVector3> points;
	
	// This is a simplified implementation
	// In a real implementation, you would:
	// 1. Sort points properly along the cross section
	// 2. Handle multiple separate features
	// 3. Apply proper geometric algorithms
	
	for ( int index : featureIndices )
	{
		// Note: This is a placeholder - in real implementation you need access to the original cloud
		// For now, create some dummy points arranged in a circle-like pattern
		double angle = 2.0 * M_PI * index / featureIndices.size();
		CCVector3 point(
			cos( angle ) * 0.1,
			sin( angle ) * 0.1,
			0.0
		);
		vertices->addPoint( point );
	}
	
	if ( vertices->size() < 2 )
	{
		delete vertices;
		m_lastError = "Insufficient points for polyline creation";
		return false;
	}
	
	ccPolyline* polyline = new ccPolyline( vertices );
	polyline->setName( "Cross Section Features" );
	polyline->addPointIndex( 0, vertices->size() );
	polyline->setClosed( true );  // Close the polyline for cross section
	
	features.push_back( polyline );
	
	return true;
}