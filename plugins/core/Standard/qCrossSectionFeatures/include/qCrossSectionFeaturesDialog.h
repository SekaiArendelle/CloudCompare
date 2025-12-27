#pragma once

// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
// #                                                                        #
// ##########################################################################

#include <QDialog>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QGroupBox>
#include <QGridLayout>

//! Dialog for cross section feature extraction parameters
class qCrossSectionFeaturesDialog : public QDialog
{
	Q_OBJECT

public:
	//! Constructor
	explicit qCrossSectionFeaturesDialog( QWidget* parent = nullptr );
	
	//! Destructor
	~qCrossSectionFeaturesDialog() override = default;
	
	//! Get plane resolution
	double getPlaneResolution() const;
	
	//! Get feature threshold
	double getFeatureThreshold() const;
	
	//! Get minimum points per feature
	int getMinPointsPerFeature() const;
	
	//! Get search radius
	double getSearchRadius() const;
	
	//! Get advanced features enabled
	bool isAdvancedFeaturesEnabled() const;
	
	//! Set plane resolution
	void setPlaneResolution( double resolution );
	
	//! Set feature threshold
	void setFeatureThreshold( double threshold );
	
	//! Set minimum points per feature
	void setMinPointsPerFeature( int points );
	
	//! Set search radius
	void setSearchRadius( double radius );
	
	//! Set advanced features enabled
	void setAdvancedFeaturesEnabled( bool enabled );

private:
	//! Handle OK button click
	void onOkClicked();
	
	//! Handle Cancel button click
	void onCancelClicked();
	
	//! Handle defaults button click
	void onDefaultsClicked();

private:
	//! Create UI elements
	void createUI();
	
	//! Connect signals and slots
	void connectSignals();
	
	//! Set default values
	void setDefaults();
	
	// UI Elements
	QDoubleSpinBox* m_planeResolutionSpinBox;
	QDoubleSpinBox* m_featureThresholdSpinBox;
	QSpinBox* m_minPointsSpinBox;
	QDoubleSpinBox* m_searchRadiusSpinBox;
	QCheckBox* m_advancedFeaturesCheckBox;
	
	QPushButton* m_okButton;
	QPushButton* m_cancelButton;
	QPushButton* m_defaultsButton;
	
	// Group boxes
	QGroupBox* m_basicGroupBox;
	QGroupBox* m_advancedGroupBox;
	
	// Main layout
	QGridLayout* m_mainLayout;
};