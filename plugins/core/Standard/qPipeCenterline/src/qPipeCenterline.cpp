// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

#include "../include/qPipeCenterline.h"
#include "../include/qPipeCenterlineDialog.h"
#include "../include/PipeCenterlineExtractor.h"

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

qPipeCenterline::qPipeCenterline( QObject* parent )
	: QObject( parent )
	, ccStdPluginInterface( ":/CC/plugin/qPipeCenterline/info.json" )
	, m_action( nullptr )
	, m_dialog( nullptr )
	, m_selectedCloud( nullptr )
{
	// Create action
	m_action = new QAction( getName(), this );
	m_action->setToolTip( getDescription() );
	m_action->setIcon( getIcon() );
	
	connect( m_action, &QAction::triggered, this, &qPipeCenterline::doAction );
}


QIcon qPipeCenterline::getIcon() const
{
	return QIcon( QStringLiteral( ":/CC/plugin/qPipeCenterline/images/qPipeCenterline.png" ) );
}

void qPipeCenterline::onNewSelection( const ccHObject::Container& selectedEntities )
{
	if ( m_action == nullptr )
	{
		return;
	}
	
	bool validSelection = false;
	
	if ( selectedEntities.size() == 1 )
	{
		ccHObject* entity = selectedEntities[0];
		if ( isValidPipeCloud( entity ) )
		{
			validSelection = true;
			m_selectedCloud = ccHObjectCaster::ToPointCloud( entity );
		}
	}
	
	m_action->setEnabled( validSelection );
}

QList<QAction*> qPipeCenterline::getActions()
{
	return QList<QAction*>() << m_action;
}

void qPipeCenterline::doAction()
{
	if ( !m_selectedCloud )
	{
		return;
	}
	
	// Create dialog if needed
	if ( !m_dialog )
	{
		m_dialog = new qPipeCenterlineDialog( m_app ? m_app->getMainWindow() : nullptr );
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
	
	// Extract centerline
	extractCenterline();
}

void qPipeCenterline::extractCenterline()
{
	if ( !m_selectedCloud )
	{
		return;
	}
	
	// Create progress dialog
	QProgressDialog progress( "Extracting pipe centerline...", "Cancel", 0, 0, m_app->getMainWindow() );
	progress.setWindowModality( Qt::WindowModal );
	progress.show();
	QApplication::processEvents();
	
	try
	{
		// Set up parameters
		PipeCenterlineExtractor::Parameters params;
		params.radiusEstimate = m_dialog->getRadiusEstimate();
		params.voxelSize = m_dialog->getVoxelSize();
		params.minPointsPerSegment = m_dialog->getMinPointsPerSegment();
		params.curvatureThreshold = m_dialog->getCurvatureThreshold();
		params.distanceThreshold = m_dialog->getDistanceThreshold();
		params.useBranchDetection = m_dialog->isBranchDetectionEnabled();
		params.branchAngleThreshold = m_dialog->getBranchAngleThreshold();
		params.useRANSAC = m_dialog->isRANSACEnabled();
		params.ransacDistanceThreshold = m_dialog->getRANSACDistanceThreshold();
		params.ransacMaxIterations = m_dialog->getRANSACMaxIterations();
		params.useMLSR = m_dialog->isMLSREnabled();
		params.mlsrSearchRadius = m_dialog->getMLSRSearchRadius();
		
		// Create extractor
		PipeCenterlineExtractor extractor( params );
		
		// Extract centerlines
		std::vector<ccPolyline*> centerlines;
		if ( !extractor.extract( m_selectedCloud, centerlines ) )
		{
			QString error = extractor.getLastError();
			QMessageBox::critical( m_app->getMainWindow(), "Error", 
				QString( "Failed to extract centerline: %1" ).arg( error ) );
			return;
		}
		
		// Render centerlines
		renderCenterlines( centerlines );
		
		// Show success message
		QMessageBox::information( m_app->getMainWindow(), "Success", 
			QString( "Successfully extracted %1 centerline segment(s)" ).arg( centerlines.size() ) );
	}
	catch ( const std::exception& e )
	{
		QMessageBox::critical( m_app->getMainWindow(), "Error", 
			QString( "Exception occurred: %1" ).arg( e.what() ) );
	}
}

bool qPipeCenterline::preprocessPointCloud( ccPointCloud* cloud, ccPointCloud*& processedCloud )
{
	if ( !cloud )
	{
		return false;
	}
	
	// For now, just create a copy
	// TODO: Implement actual preprocessing (filtering, denoising, etc.)
	processedCloud = new ccPointCloud( cloud->getName() + "_processed" );
	*processedCloud = *cloud;
	
	return true;
}

bool qPipeCenterline::isValidPipeCloud( ccHObject* entity )
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

ccPolyline* qPipeCenterline::createPolyline( const std::vector<CCVector3>& points, 
											 const QString& name )
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

void qPipeCenterline::renderCenterlines( const std::vector<ccPolyline*>& centerlines )
{
	if ( centerlines.empty() || !m_app )
	{
		return;
	}
	
	// Add centerlines to the main DB
	ccHObject* root = m_app->dbRootObject();
	if ( !root )
	{
		return;
	}
	
	// Create a group for centerlines
	ccHObject* centerlineGroup = new ccHObject( "Pipe Centerlines" );
	centerlineGroup->setDisplay( m_selectedCloud->getDisplay() );
	
	for ( size_t i = 0; i < centerlines.size(); ++i )
	{
		ccPolyline* centerline = centerlines[i];
		if ( centerline )
		{
			centerline->setName( QString( "Centerline %1" ).arg( i + 1 ) );
			centerline->setVisible( true );
			centerline->setDisplay( m_selectedCloud->getDisplay() );
			
			// Set color (rainbow colors for different segments)
			ccColor::Rgb col = ccColor::Generator::Random();
			centerline->setColor( col );
			centerline->showColors( true );
			
			// Set line width
			centerline->setWidth( 3.0 );
			
			centerlineGroup->addChild( centerline );
		}
	}
	
	root->addChild( centerlineGroup );
	
	// Refresh display
	m_app->addToDB( centerlineGroup );
	m_app->refreshAll();
	m_app->updateUI();
}