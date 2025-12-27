// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
// #                                                                        #
// ##########################################################################

#include "../include/qCrossSectionFeatures.h"
#include "../include/qCrossSectionFeaturesDialog.h"
#include "../include/CrossSectionExtractor.h"

// Qt
#include <QIcon>
#include <QMenu>
#include <QMessageBox>
#include <QMainWindow>
#include <QProgressDialog>
#include <QApplication>

// qCC
#include "ccMainAppInterface.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccHObjectCaster.h"

// CCCoreLib
#include <ccHObject.h>

qCrossSectionFeatures::qCrossSectionFeatures( QObject* parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qCrossSectionFeatures/info.json" )
	, m_action( nullptr )
	, m_dialog( nullptr )
	, m_selectedCloud( nullptr )
{
	// Create action
	m_action = new QAction( getName(), this );
	m_action->setToolTip( getDescription() );
	m_action->setIcon( getIcon() );
	
	connect( m_action, &QAction::triggered, this, &qCrossSectionFeatures::doAction );
}

QIcon qCrossSectionFeatures::getIcon() const
{
	return QIcon( QStringLiteral( ":/CC/plugin/qCrossSectionFeatures/images/qCrossSectionFeatures.png" ) );
}

void qCrossSectionFeatures::onNewSelection( const ccHObject::Container& selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	bool validSelection = false;
	
	if ( selectedEntities.size() == 1 )
	{
		ccHObject* entity = selectedEntities[0];
		if ( isValidCrossSectionCloud( entity ) )
		{
			validSelection = true;
			m_selectedCloud = ccHObjectCaster::ToPointCloud( entity );
		}
	}
	
	m_action->setEnabled( validSelection );
}

QList<QAction*> qCrossSectionFeatures::getActions()
{
	return QList<QAction*>() << m_action;
}

void qCrossSectionFeatures::doAction()
{
	if ( !m_selectedCloud )
	{
		return;
	}
	
	// Create dialog if needed
	if ( !m_dialog )
	{
		m_dialog = new qCrossSectionFeaturesDialog( m_app ? m_app->getMainWindow() : nullptr );
	}
	
	if ( !m_dialog )
	{
		return;
	}
	
	// Show dialog and get parameters
	if ( m_dialog->exec() != QDialog::Accepted )
	{
		return;
	}
	
	// Extract features
	extractFeatures();
}

bool qCrossSectionFeatures::isValidCrossSectionCloud( ccHObject* entity )
{
	if ( !entity )
	{
		return false;
	}
	
	// Check if it's a point cloud
	if ( !entity->isA( CC_TYPES::POINT_CLOUD ) )
	{
		return false;
	}
	
	ccPointCloud* cloud = ccHObjectCaster::ToPointCloud( entity );
	if ( !cloud )
	{
		return false;
	}
	
	// Check minimum number of points
	if ( cloud->size() < 100 )
	{
		return false;
	}
	
	return true;
}

ccPolyline* qCrossSectionFeatures::createPolyline( const std::vector<CCVector3>& points, const QString& name )
{
	if ( points.empty() )
	{
		return nullptr;
	}
	
	ccPointCloud* vertices = new ccPointCloud( "vertices" );
	for ( const auto& point : points )
	{
		vertices->addPoint( point );
	}
	
	ccPolyline* polyline = new ccPolyline( vertices );
	polyline->setName( name );
	polyline->addPointIndex( 0, static_cast<unsigned>( points.size() ) );
	polyline->setClosed( false );
	
	return polyline;
}

void qCrossSectionFeatures::extractFeatures()
{
	if ( !m_selectedCloud )
	{
		return;
	}
	
	// Create progress dialog
	QProgressDialog progress( "Extracting cross section features...", "Cancel", 0, 0, m_app->getMainWindow() );
	progress.setWindowModality( Qt::WindowModal );
	progress.show();
	QApplication::processEvents();
	
	try
	{
		// Set up parameters
		CrossSectionExtractor::Parameters params;
		params.planeResolution = m_dialog->getPlaneResolution();
		params.featureThreshold = m_dialog->getFeatureThreshold();
		params.minPointsPerFeature = m_dialog->getMinPointsPerFeature();
		params.searchRadius = m_dialog->getSearchRadius();
		params.useAdvancedFeatures = m_dialog->isAdvancedFeaturesEnabled();
		
		// Create extractor
		CrossSectionExtractor extractor( params );
		
		// Extract features
		std::vector<ccPolyline*> features;
		if ( !extractor.extract( m_selectedCloud, features ) )
		{
			QString error = extractor.getLastError();
			QMessageBox::critical( m_app->getMainWindow(), "Error", 
				QString( "Failed to extract features: %1" ).arg( error ) );
			return;
		}
		
		// Render features (simplified for now)
		if ( !features.empty() )
		{
			QMessageBox::information( m_app->getMainWindow(), "Success", 
				QString( "Successfully extracted %1 feature(s)" ).arg( features.size() ) );
		}
		else
		{
			QMessageBox::information( m_app->getMainWindow(), "Info", 
				"No features were detected in the cross section." );
		}
	}
	catch ( const std::exception& e )
	{
		QMessageBox::critical( m_app->getMainWindow(), "Error", 
			QString( "Exception occurred: %1" ).arg( e.what() ) );
	}
}