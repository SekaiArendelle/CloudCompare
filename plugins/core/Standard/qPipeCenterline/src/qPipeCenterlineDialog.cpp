// Simplified dialog implementation for Open3D parameters
// This provides basic access to Open3D parameters without full UI

#include "../include/qPipeCenterlineDialog.h"

// Qt
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

qPipeCenterlineDialog::qPipeCenterlineDialog(QWidget* parent)
    : QDialog(parent)
{
	setWindowTitle("Pipe Centerline Extraction Parameters");
	setModal(true);
	resize(400, 500);

	createUI();
	connectSignals();
	setDefaults();
}

void qPipeCenterlineDialog::createUI()
{
	// Basic parameters group
	m_basicGroupBox  = new QGroupBox("Basic Parameters", this);
	auto basicLayout = new QGridLayout(m_basicGroupBox);

	basicLayout->addWidget(new QLabel("Estimated pipe radius:"), 0, 0);
	m_radiusEstimateSpinBox = new QDoubleSpinBox(this);
	m_radiusEstimateSpinBox->setRange(0.001, 10.0);
	m_radiusEstimateSpinBox->setSingleStep(0.01);
	m_radiusEstimateSpinBox->setValue(0.1);
	basicLayout->addWidget(m_radiusEstimateSpinBox, 0, 1);

	basicLayout->addWidget(new QLabel("Voxel size:"), 1, 0);
	m_voxelSizeSpinBox = new QDoubleSpinBox(this);
	m_voxelSizeSpinBox->setRange(0.001, 1.0);
	m_voxelSizeSpinBox->setSingleStep(0.001);
	m_voxelSizeSpinBox->setValue(0.02);
	basicLayout->addWidget(m_voxelSizeSpinBox, 1, 1);

	basicLayout->addWidget(new QLabel("Min points per segment:"), 2, 0);
	m_minPointsSpinBox = new QSpinBox(this);
	m_minPointsSpinBox->setRange(10, 1000);
	m_minPointsSpinBox->setSingleStep(10);
	m_minPointsSpinBox->setValue(50);
	basicLayout->addWidget(m_minPointsSpinBox, 2, 1);

	// Advanced parameters group
	m_advancedGroupBox  = new QGroupBox("Advanced Parameters", this);
	auto advancedLayout = new QGridLayout(m_advancedGroupBox);

	advancedLayout->addWidget(new QLabel("Curvature threshold:"), 0, 0);
	m_curvatureThresholdSpinBox = new QDoubleSpinBox(this);
	m_curvatureThresholdSpinBox->setRange(0.01, 1.0);
	m_curvatureThresholdSpinBox->setSingleStep(0.01);
	m_curvatureThresholdSpinBox->setValue(0.1);
	advancedLayout->addWidget(m_curvatureThresholdSpinBox, 0, 1);

	advancedLayout->addWidget(new QLabel("Distance threshold:"), 1, 0);
	m_distanceThresholdSpinBox = new QDoubleSpinBox(this);
	m_distanceThresholdSpinBox->setRange(0.001, 1.0);
	m_distanceThresholdSpinBox->setSingleStep(0.001);
	m_distanceThresholdSpinBox->setValue(0.05);
	advancedLayout->addWidget(m_distanceThresholdSpinBox, 1, 1);

	// Branch detection group
	m_branchGroupBox  = new QGroupBox("Branch Detection", this);
	auto branchLayout = new QGridLayout(m_branchGroupBox);

	m_branchDetectionCheckBox = new QCheckBox("Enable branch detection", this);
	m_branchDetectionCheckBox->setChecked(true);
	branchLayout->addWidget(m_branchDetectionCheckBox, 0, 0, 1, 2);

	branchLayout->addWidget(new QLabel("Branch angle threshold:"), 1, 0);
	m_branchAngleSpinBox = new QDoubleSpinBox(this);
	m_branchAngleSpinBox->setRange(5.0, 90.0);
	m_branchAngleSpinBox->setSingleStep(1.0);
	m_branchAngleSpinBox->setValue(30.0);
	branchLayout->addWidget(m_branchAngleSpinBox, 1, 1);

	// Open3D parameters group (optional)
	m_open3dGroupBox  = new QGroupBox("Open3D Parameters (if available)", this);
	auto open3dLayout = new QGridLayout(m_open3dGroupBox);

	m_ransacCheckBox = new QCheckBox("Use RANSAC cylinder detection", this);
	m_ransacCheckBox->setChecked(true);
	open3dLayout->addWidget(m_ransacCheckBox, 0, 0, 1, 2);

	open3dLayout->addWidget(new QLabel("RANSAC distance threshold:"), 1, 0);
	m_ransacDistanceSpinBox = new QDoubleSpinBox(this);
	m_ransacDistanceSpinBox->setRange(0.001, 0.1);
	m_ransacDistanceSpinBox->setSingleStep(0.001);
	m_ransacDistanceSpinBox->setValue(0.01);
	open3dLayout->addWidget(m_ransacDistanceSpinBox, 1, 1);

	open3dLayout->addWidget(new QLabel("RANSAC max iterations:"), 2, 0);
	m_ransacIterationsSpinBox = new QSpinBox(this);
	m_ransacIterationsSpinBox->setRange(100, 10000);
	m_ransacIterationsSpinBox->setSingleStep(100);
	m_ransacIterationsSpinBox->setValue(1000);
	open3dLayout->addWidget(m_ransacIterationsSpinBox, 2, 1);

	m_mlsrCheckBox = new QCheckBox("Use MLS smoothing", this);
	m_mlsrCheckBox->setChecked(true);
	open3dLayout->addWidget(m_mlsrCheckBox, 3, 0, 1, 2);

	open3dLayout->addWidget(new QLabel("MLSR search radius:"), 4, 0);
	m_mlsrRadiusSpinBox = new QDoubleSpinBox(this);
	m_mlsrRadiusSpinBox->setRange(0.001, 0.5);
	m_mlsrRadiusSpinBox->setSingleStep(0.001);
	m_mlsrRadiusSpinBox->setValue(0.05);
	open3dLayout->addWidget(m_mlsrRadiusSpinBox, 4, 1);

	// Buttons
	m_okButton       = new QPushButton("OK", this);
	m_cancelButton   = new QPushButton("Cancel", this);
	m_defaultsButton = new QPushButton("Defaults", this);

	auto buttonLayout = new QHBoxLayout();
	buttonLayout->addWidget(m_defaultsButton);
	buttonLayout->addStretch();
	buttonLayout->addWidget(m_cancelButton);
	buttonLayout->addWidget(m_okButton);

	// Main layout
	m_mainLayout = new QVBoxLayout(this);
	m_mainLayout->addWidget(m_basicGroupBox);
	m_mainLayout->addWidget(m_advancedGroupBox);
	m_mainLayout->addWidget(m_branchGroupBox);
	m_mainLayout->addWidget(m_open3dGroupBox);
	m_mainLayout->addLayout(buttonLayout);

	setLayout(m_mainLayout);
}

void qPipeCenterlineDialog::connectSignals()
{
	connect(m_okButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onOkClicked);
	connect(m_cancelButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onCancelClicked);
	connect(m_defaultsButton, &QPushButton::clicked, this, &qPipeCenterlineDialog::onDefaultsClicked);

	connect(m_branchDetectionCheckBox, &QCheckBox::toggled, this, &qPipeCenterlineDialog::updateUI);
	connect(m_ransacCheckBox, &QCheckBox::toggled, this, &qPipeCenterlineDialog::onRANSACChanged);
	connect(m_mlsrCheckBox, &QCheckBox::toggled, this, &qPipeCenterlineDialog::onMLSRChanged);
}

void qPipeCenterlineDialog::setDefaults()
{
	m_radiusEstimateSpinBox->setValue(0.1);
	m_voxelSizeSpinBox->setValue(0.02);
	m_minPointsSpinBox->setValue(50);
	m_curvatureThresholdSpinBox->setValue(0.1);
	m_distanceThresholdSpinBox->setValue(0.05);
	m_branchDetectionCheckBox->setChecked(true);
	m_branchAngleSpinBox->setValue(30.0);

	m_ransacCheckBox->setChecked(true);
	m_ransacDistanceSpinBox->setValue(0.01);
	m_ransacIterationsSpinBox->setValue(1000);
	m_mlsrCheckBox->setChecked(true);
	m_mlsrRadiusSpinBox->setValue(0.05);

	updateUI();
}

void qPipeCenterlineDialog::updateUI()
{
	bool branchDetection = m_branchDetectionCheckBox->isChecked();
	m_branchAngleSpinBox->setEnabled(branchDetection);

	bool ransacEnabled = m_ransacCheckBox->isChecked();
	m_ransacDistanceSpinBox->setEnabled(ransacEnabled);
	m_ransacIterationsSpinBox->setEnabled(ransacEnabled);

	bool mlsrEnabled = m_mlsrCheckBox->isChecked();
	m_mlsrRadiusSpinBox->setEnabled(mlsrEnabled);
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

void qPipeCenterlineDialog::onRANSACChanged()
{
	updateUI();
}

void qPipeCenterlineDialog::onMLSRChanged()
{
	updateUI();
}

// Parameter getters
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

// Open3D parameter getters
bool qPipeCenterlineDialog::isRANSACEnabled() const
{
	return m_ransacCheckBox ? m_ransacCheckBox->isChecked() : true;
}

double qPipeCenterlineDialog::getRANSACDistanceThreshold() const
{
	return m_ransacDistanceSpinBox ? m_ransacDistanceSpinBox->value() : 0.01;
}

int qPipeCenterlineDialog::getRANSACMaxIterations() const
{
	return m_ransacIterationsSpinBox ? m_ransacIterationsSpinBox->value() : 1000;
}

bool qPipeCenterlineDialog::isMLSREnabled() const
{
	return m_mlsrCheckBox ? m_mlsrCheckBox->isChecked() : true;
}

double qPipeCenterlineDialog::getMLSRSearchRadius() const
{
	return m_mlsrRadiusSpinBox ? m_mlsrRadiusSpinBox->value() : 0.05;
}

// Parameter setters
void qPipeCenterlineDialog::setRadiusEstimate(double radius)
{
	if (m_radiusEstimateSpinBox)
		m_radiusEstimateSpinBox->setValue(radius);
}

void qPipeCenterlineDialog::setVoxelSize(double size)
{
	if (m_voxelSizeSpinBox)
		m_voxelSizeSpinBox->setValue(size);
}

void qPipeCenterlineDialog::setMinPointsPerSegment(int points)
{
	if (m_minPointsSpinBox)
		m_minPointsSpinBox->setValue(points);
}

void qPipeCenterlineDialog::setCurvatureThreshold(double threshold)
{
	if (m_curvatureThresholdSpinBox)
		m_curvatureThresholdSpinBox->setValue(threshold);
}

void qPipeCenterlineDialog::setDistanceThreshold(double threshold)
{
	if (m_distanceThresholdSpinBox)
		m_distanceThresholdSpinBox->setValue(threshold);
}

void qPipeCenterlineDialog::setBranchDetectionEnabled(bool enabled)
{
	if (m_branchDetectionCheckBox)
		m_branchDetectionCheckBox->setChecked(enabled);
}

void qPipeCenterlineDialog::setBranchAngleThreshold(double angle)
{
	if (m_branchAngleSpinBox)
		m_branchAngleSpinBox->setValue(angle);
}

void qPipeCenterlineDialog::setRANSACEnabled(bool enabled)
{
	if (m_ransacCheckBox)
		m_ransacCheckBox->setChecked(enabled);
}

void qPipeCenterlineDialog::setRANSACDistanceThreshold(double threshold)
{
	if (m_ransacDistanceSpinBox)
		m_ransacDistanceSpinBox->setValue(threshold);
}

void qPipeCenterlineDialog::setRANSACMaxIterations(int iterations)
{
	if (m_ransacIterationsSpinBox)
		m_ransacIterationsSpinBox->setValue(iterations);
}

void qPipeCenterlineDialog::setMLSREnabled(bool enabled)
{
	if (m_mlsrCheckBox)
		m_mlsrCheckBox->setChecked(enabled);
}

void qPipeCenterlineDialog::setMLSRSearchRadius(double radius)
{
	if (m_mlsrRadiusSpinBox)
		m_mlsrRadiusSpinBox->setValue(radius);
}