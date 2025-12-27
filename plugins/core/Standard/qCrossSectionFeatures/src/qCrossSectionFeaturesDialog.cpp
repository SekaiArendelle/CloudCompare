// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
// #                                                                        #
// ##########################################################################

#include "../include/qCrossSectionFeaturesDialog.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QDialogButtonBox>

qCrossSectionFeaturesDialog::qCrossSectionFeaturesDialog( QWidget* parent )
	: QDialog( parent )
{
	setWindowTitle( "Cross Section Features" );
	setModal( true );
	resize( 400, 400 );
	
	createUI();
	connectSignals();
	setDefaults();
}

void qCrossSectionFeaturesDialog::createUI()
{
	// Main layout
	QVBoxLayout* mainLayout = new QVBoxLayout( this );
	
	// Basic parameters group
	m_basicGroupBox = new QGroupBox( "Basic Parameters", this );
	QGridLayout* basicLayout = new QGridLayout( m_basicGroupBox );
	
	// Plane resolution
	QLabel* planeResLabel = new QLabel( "Plane Resolution:", m_basicGroupBox );
	m_planeResolutionSpinBox = new QDoubleSpinBox( m_basicGroupBox );
	m_planeResolutionSpinBox->setRange( 0.001, 1.0 );
	m_planeResolutionSpinBox->setDecimals( 3 );
	m_planeResolutionSpinBox->setSuffix( " units" );
	m_planeResolutionSpinBox->setToolTip( "Resolution for plane fitting" );
	
	basicLayout->addWidget( planeResLabel, 0, 0 );
	basicLayout->addWidget( m_planeResolutionSpinBox, 0, 1 );
	
	// Feature threshold
	QLabel* thresholdLabel = new QLabel( "Feature Threshold:", m_basicGroupBox );
	m_featureThresholdSpinBox = new QDoubleSpinBox( m_basicGroupBox );
	m_featureThresholdSpinBox->setRange( 0.001, 1.0 );
	m_featureThresholdSpinBox->setDecimals( 3 );
	m_featureThresholdSpinBox->setSingleStep( 0.01 );
	m_featureThresholdSpinBox->setToolTip( "Threshold for feature detection" );
	
	basicLayout->addWidget( thresholdLabel, 1, 0 );
	basicLayout->addWidget( m_featureThresholdSpinBox, 1, 1 );
	
	// Min points per feature
	QLabel* minPointsLabel = new QLabel( "Min Points per Feature:", m_basicGroupBox );
	m_minPointsSpinBox = new QSpinBox( m_basicGroupBox );
	m_minPointsSpinBox->setRange( 5, 1000 );
	m_minPointsSpinBox->setSingleStep( 5 );
	m_minPointsSpinBox->setToolTip( "Minimum number of points required for a feature" );
	
	basicLayout->addWidget( minPointsLabel, 2, 0 );
	basicLayout->addWidget( m_minPointsSpinBox, 2, 1 );
	
	// Search radius
	QLabel* searchRadiusLabel = new QLabel( "Search Radius:", m_basicGroupBox );
	m_searchRadiusSpinBox = new QDoubleSpinBox( m_basicGroupBox );
	m_searchRadiusSpinBox->setRange( 0.001, 1.0 );
	m_searchRadiusSpinBox->setDecimals( 3 );
	m_searchRadiusSpinBox->setSuffix( " units" );
	m_searchRadiusSpinBox->setToolTip( "Search radius for neighbors" );
	
	basicLayout->addWidget( searchRadiusLabel, 3, 0 );
	basicLayout->addWidget( m_searchRadiusSpinBox, 3, 1 );
	
	mainLayout->addWidget( m_basicGroupBox );
	
	// Advanced parameters group
	m_advancedGroupBox = new QGroupBox( "Advanced Parameters", this );
	QGridLayout* advancedLayout = new QGridLayout( m_advancedGroupBox );
	
	// Advanced features
	m_advancedFeaturesCheckBox = new QCheckBox( "Enable Advanced Features", m_advancedGroupBox );
	m_advancedFeaturesCheckBox->setToolTip( "Enable advanced feature detection algorithms" );
	
	advancedLayout->addWidget( m_advancedFeaturesCheckBox, 0, 0, 1, 2 );
	
	mainLayout->addWidget( m_advancedGroupBox );
	
	// Button box
	QDialogButtonBox* buttonBox = new QDialogButtonBox( this );
	
	m_defaultsButton = new QPushButton( "Defaults", buttonBox );
	m_okButton = new QPushButton( "OK", buttonBox );
	m_cancelButton = new QPushButton( "Cancel", buttonBox );
	
	buttonBox->addButton( m_defaultsButton, QDialogButtonBox::ActionRole );
	buttonBox->addButton( m_okButton, QDialogButtonBox::AcceptRole );
	buttonBox->addButton( m_cancelButton, QDialogButtonBox::RejectRole );
	
	mainLayout->addWidget( buttonBox );
}

void qCrossSectionFeaturesDialog::connectSignals()
{
	connect( m_okButton, &QPushButton::clicked, this, &qCrossSectionFeaturesDialog::onOkClicked );
	connect( m_cancelButton, &QPushButton::clicked, this, &qCrossSectionFeaturesDialog::onCancelClicked );
	connect( m_defaultsButton, &QPushButton::clicked, this, &qCrossSectionFeaturesDialog::onDefaultsClicked );
}

void qCrossSectionFeaturesDialog::setDefaults()
{
	m_planeResolutionSpinBox->setValue( 0.01 );
	m_featureThresholdSpinBox->setValue( 0.1 );
	m_minPointsSpinBox->setValue( 10 );
	m_searchRadiusSpinBox->setValue( 0.05 );
	m_advancedFeaturesCheckBox->setChecked( false );
}

void qCrossSectionFeaturesDialog::onOkClicked()
{
	accept();
}

void qCrossSectionFeaturesDialog::onCancelClicked()
{
	reject();
}

void qCrossSectionFeaturesDialog::onDefaultsClicked()
{
	setDefaults();
}

double qCrossSectionFeaturesDialog::getPlaneResolution() const
{
	return m_planeResolutionSpinBox->value();
}

double qCrossSectionFeaturesDialog::getFeatureThreshold() const
{
	return m_featureThresholdSpinBox->value();
}

int qCrossSectionFeaturesDialog::getMinPointsPerFeature() const
{
	return m_minPointsSpinBox->value();
}

double qCrossSectionFeaturesDialog::getSearchRadius() const
{
	return m_searchRadiusSpinBox->value();
}

bool qCrossSectionFeaturesDialog::isAdvancedFeaturesEnabled() const
{
	return m_advancedFeaturesCheckBox->isChecked();
}

void qCrossSectionFeaturesDialog::setPlaneResolution( double resolution )
{
	m_planeResolutionSpinBox->setValue( resolution );
}

void qCrossSectionFeaturesDialog::setFeatureThreshold( double threshold )
{
	m_featureThresholdSpinBox->setValue( threshold );
}

void qCrossSectionFeaturesDialog::setMinPointsPerFeature( int points )
{
	m_minPointsSpinBox->setValue( points );
}

void qCrossSectionFeaturesDialog::setSearchRadius( double radius )
{
	m_searchRadiusSpinBox->setValue( radius );
}

void qCrossSectionFeaturesDialog::setAdvancedFeaturesEnabled( bool enabled )
{
	m_advancedFeaturesCheckBox->setChecked( enabled );
}