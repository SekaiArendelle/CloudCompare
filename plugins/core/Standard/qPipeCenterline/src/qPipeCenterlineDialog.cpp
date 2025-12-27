// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

#include "../include/qPipeCenterlineDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QDialogButtonBox>

qPipeCenterlineDialog::qPipeCenterlineDialog( QWidget* parent )
	: QDialog( parent )
{
	setWindowTitle( "Pipe Centerline Extraction" );
	setModal( true );
	resize( 400, 500 );
	
	createUI();
	connectSignals();
	setDefaults();
}

void qPipeCenterlineDialog::createUI()
{
	// Main layout
	m_mainLayout = new QGridLayout( this );
	
	// Basic parameters group
	m_basicGroupBox = new QGroupBox( "Basic Parameters", this );
	QGridLayout* basicLayout = new QGridLayout( m_basicGroupBox );
	
	// Radius estimate
	QLabel* radiusLabel = new QLabel( "Estimated Pipe Radius:", m_basicGroupBox );
	m_radiusEstimateSpinBox = new QDoubleSpinBox( m_basicGroupBox );
	m_radiusEstimateSpinBox->setRange( 0.001, 1000.0 );
	m_radiusEstimateSpinBox->setDecimals( 3 );
	m_radiusEstimateSpinBox->setSuffix( " units" );
	m_radiusEstimateSpinBox->setToolTip( "Estimated radius of the pipe (used for feature detection)" );
	
	basicLayout->addWidget( radiusLabel, 0, 0 );
	basicLayout->addWidget( m_radiusEstimateSpinBox, 0, 1 );
	
	// Voxel size
	QLabel* voxelLabel = new QLabel( "Voxel Size:", m_basicGroupBox );
	m_voxelSizeSpinBox = new QDoubleSpinBox( m_basicGroupBox );
	m_voxelSizeSpinBox->setRange( 0.001, 100.0 );
	m_voxelSizeSpinBox->setDecimals( 3 );
	m_voxelSizeSpinBox->setSuffix( " units" );
	m_voxelSizeSpinBox->setToolTip( "Size of voxel grid for downsampling (smaller = more detail, slower)" );
	
	basicLayout->addWidget( voxelLabel, 1, 0 );
	basicLayout->addWidget( m_voxelSizeSpinBox, 1, 1 );
	
	// Min points per segment
	QLabel* minPointsLabel = new QLabel( "Min Points per Segment:", m_basicGroupBox );
	m_minPointsSpinBox = new QSpinBox( m_basicGroupBox );
	m_minPointsSpinBox->setRange( 10, 10000 );
	m_minPointsSpinBox->setSingleStep( 10 );
	m_minPointsSpinBox->setToolTip( "Minimum number of points required for a valid centerline segment" );
	
	basicLayout->addWidget( minPointsLabel, 2, 0 );
	basicLayout->addWidget( m_minPointsSpinBox, 2, 1 );
	
	m_mainLayout->addWidget( m_basicGroupBox, 0, 0, 1, 2 );
	
	// Advanced parameters group
	m_advancedGroupBox = new QGroupBox( "Advanced Parameters", this );
	QGridLayout* advancedLayout = new QGridLayout( m_advancedGroupBox );
	
	// Curvature threshold
	QLabel* curvatureLabel = new QLabel( "Curvature Threshold:", m_advancedGroupBox );
	m_curvatureThresholdSpinBox = new QDoubleSpinBox( m_advancedGroupBox );
	m_curvatureThresholdSpinBox->setRange( 0.001, 1.0 );
	m_curvatureThresholdSpinBox->setDecimals( 3 );
	m_curvatureThresholdSpinBox->setSingleStep( 0.01 );
	m_curvatureThresholdSpinBox->setToolTip( "Threshold for detecting pipe surface points based on curvature" );
	
	advancedLayout->addWidget( curvatureLabel, 0, 0 );
	advancedLayout->addWidget( m_curvatureThresholdSpinBox, 0, 1 );
	
	// Distance threshold
	QLabel* distanceLabel = new QLabel( "Distance Threshold:", m_advancedGroupBox );
	m_distanceThresholdSpinBox = new QDoubleSpinBox( m_advancedGroupBox );
	m_distanceThresholdSpinBox->setRange( 0.001, 100.0 );
	m_distanceThresholdSpinBox->setDecimals( 3 );
	m_distanceThresholdSpinBox->setSuffix( " units" );
	m_distanceThresholdSpinBox->setToolTip( "Distance threshold for point clustering" );
	
	advancedLayout->addWidget( distanceLabel, 1, 0 );
	advancedLayout->addWidget( m_distanceThresholdSpinBox, 1, 1 );
	
	m_mainLayout->addWidget( m_advancedGroupBox, 1, 0, 1, 2 );
	
	// Branch detection group
	m_branchGroupBox = new QGroupBox( "Branch Detection", this );
	QGridLayout* branchLayout = new QGridLayout( m_branchGroupBox );
	
	// Enable branch detection
	m_branchDetectionCheckBox = new QCheckBox( "Enable Branch Detection", m_branchGroupBox );
	m_branchDetectionCheckBox->setToolTip( "Enable detection and handling of pipe branches and bifurcations" );
	
	branchLayout->addWidget( m_branchDetectionCheckBox, 0, 0, 1, 2 );
	
	// Branch angle threshold
	QLabel* branchAngleLabel = new QLabel( "Branch Angle Threshold:", m_branchGroupBox );
	m_branchAngleSpinBox = new QDoubleSpinBox( m_branchGroupBox );
	m_branchAngleSpinBox->setRange( 5.0, 90.0 );
	m_branchAngleSpinBox->setDecimals( 1 );
	m_branchAngleSpinBox->setSuffix( " degrees" );
	m_branchAngleSpinBox->setToolTip( "Maximum angle for detecting pipe branches" );
	
	branchLayout->addWidget( branchAngleLabel, 1, 0 );
	branchLayout->addWidget( m_branchAngleSpinBox, 1, 1 );
	
	m_mainLayout->addWidget( m_branchGroupBox, 2, 0, 1, 2 );
	
	// Button box
	QDialogButtonBox* buttonBox = new QDialogButtonBox( this );
	
	m_defaultsButton = new QPushButton( "Defaults", buttonBox );
	m_okButton = new QPushButton( "OK", buttonBox );
	m_cancelButton = new QPushButton( "Cancel", buttonBox );
	
	buttonBox->addButton( m_defaultsButton, QDialogButtonBox::ActionRole );
	buttonBox->addButton( m_okButton, QDialogButtonBox::AcceptRole );
	buttonBox->addButton( m_cancelButton, QDialogButtonBox::RejectRole );
	
	m_mainLayout->addWidget( buttonBox, 3, 0, 1, 2 );
}

void qPipeCenterlineDialog::connectSignals()
{
	connect( m_okButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onOkClicked );
	connect( m_cancelButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onCancelClicked );
	connect( m_defaultsButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onDefaultsClicked );
	connect( m_branchDetectionCheckBox, &QCheckBox::toggled, this, &qPipeCenterlineDialog::updateUI );
}

void qPipeCenterlineDialog::setDefaults()
{
	m_radiusEstimateSpinBox->setValue( 0.1 );
	m_voxelSizeSpinBox->setValue( 0.02 );
	m_minPointsSpinBox->setValue( 50 );
	m_curvatureThresholdSpinBox->setValue( 0.1 );
	m_distanceThresholdSpinBox->setValue( 0.05 );
	m_branchDetectionCheckBox->setChecked( true );
	m_branchAngleSpinBox->setValue( 30.0 );
	
	updateUI();
}

void qPipeCenterlineDialog::updateUI()
{
	bool branchDetectionEnabled = m_branchDetectionCheckBox->isChecked();
	m_branchAngleSpinBox->setEnabled( branchDetectionEnabled );
}

void qPipeCenterlineDialog::onOkClicked()
{
	accept();
}

void qPipeCenterlineDialog::onCancelClicked()
{
	reject();
}

void qPipeCenterlineDialog::onDefaultsClicked()
{
	setDefaults();
}

double qPipeCenterlineDialog::getRadiusEstimate() const
{
	return m_radiusEstimateSpinBox->value();
}

double qPipeCenterlineDialog::getVoxelSize() const
{
	return m_voxelSizeSpinBox->value();
}
int qPipeCenterlineDialog::getMinPointsPerSegment() const
{
	return m_minPointsSpinBox->value();
}

double qPipeCenterlineDialog::getCurvatureThreshold() const
{
	return m_curvatureThresholdSpinBox->value();
}

double qPipeCenterlineDialog::getDistanceThreshold() const
{
	return m_distanceThresholdSpinBox->value();
}

bool qPipeCenterlineDialog::isBranchDetectionEnabled() const
{
	return m_branchDetectionCheckBox->isChecked();
}

double qPipeCenterlineDialog::getBranchAngleThreshold() const
{
	return m_branchAngleSpinBox->value();
}

void qPipeCenterlineDialog::setRadiusEstimate( double radius )
{
	m_radiusEstimateSpinBox->setValue( radius );
}

void qPipeCenterlineDialog::setVoxelSize( double size )
{
	m_voxelSizeSpinBox->setValue( size );
}

void qPipeCenterlineDialog::setMinPointsPerSegment( int points )
{
	m_minPointsSpinBox->setValue( points );
}

void qPipeCenterlineDialog::setCurvatureThreshold( double threshold )
{
	m_curvatureThresholdSpinBox->setValue( threshold );
}

void qPipeCenterlineDialog::setDistanceThreshold( double threshold )
{
	m_distanceThresholdSpinBox->setValue( threshold );
}

void qPipeCenterlineDialog::setBranchDetectionEnabled( bool enabled )
{
	m_branchDetectionCheckBox->setChecked( enabled );
}

void qPipeCenterlineDialog::setBranchAngleThreshold( double angle )
{
	m_branchAngleSpinBox->setValue( angle );
}