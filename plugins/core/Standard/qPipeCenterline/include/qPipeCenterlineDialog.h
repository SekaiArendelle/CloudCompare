#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
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

//! Dialog for pipe centerline extraction parameters
class qPipeCenterlineDialog : public QDialog
{
	Q_OBJECT

public:
	//! Constructor
	explicit qPipeCenterlineDialog( QWidget* parent = nullptr );
	
	//! Destructor
	~qPipeCenterlineDialog() override = default;
	
	//! Get estimated pipe radius
	double getRadiusEstimate() const;
	
	//! Get voxel size
	double getVoxelSize() const;
	
	//! Get minimum points per segment
	int getMinPointsPerSegment() const;
	
	//! Get curvature threshold
	double getCurvatureThreshold() const;
	
	//! Get distance threshold
	double getDistanceThreshold() const;
	
	//! Get branch detection enabled
	bool isBranchDetectionEnabled() const;
	
	//! Get branch angle threshold
	double getBranchAngleThreshold() const;
	
	//! Set estimated pipe radius
	void setRadiusEstimate( double radius );
	
	//! Set voxel size
	void setVoxelSize( double size );
	
	//! Set minimum points per segment
	void setMinPointsPerSegment( int points );
	
	//! Set curvature threshold
	void setCurvatureThreshold( double threshold );
	
	//! Set distance threshold
	void setDistanceThreshold( double threshold );
	
	//! Set branch detection enabled
	void setBranchDetectionEnabled( bool enabled );
	
	//! Set branch angle threshold
	void setBranchAngleThreshold( double angle );

private:
	//! Handle OK button click
	void onOkClicked();
	
	//! Handle Cancel button click
	void onCancelClicked();
	
	//! Handle defaults button click
	void onDefaultsClicked();
	
	//! Update UI state based on checkboxes
	void updateUI();

private:
	//! Create UI elements
	void createUI();
	
	//! Connect signals and slots
	void connectSignals();
	
	//! Set default values
	void setDefaults();
	
	// UI Elements
	QDoubleSpinBox* m_radiusEstimateSpinBox;
	QDoubleSpinBox* m_voxelSizeSpinBox;
	QSpinBox* m_minPointsSpinBox;
	QDoubleSpinBox* m_curvatureThresholdSpinBox;
	QDoubleSpinBox* m_distanceThresholdSpinBox;
	QCheckBox* m_branchDetectionCheckBox;
	QDoubleSpinBox* m_branchAngleSpinBox;
	
	QPushButton* m_okButton;
	QPushButton* m_cancelButton;
	QPushButton* m_defaultsButton;
	
	// Group boxes
	QGroupBox* m_basicGroupBox;
	QGroupBox* m_advancedGroupBox;
	QGroupBox* m_branchGroupBox;
	
	// Main layout
	QGridLayout* m_mainLayout;
};