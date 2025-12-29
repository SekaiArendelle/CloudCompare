// Open3D enhanced dialog implementation
// This file contains the Open3D-specific dialog methods

#include "../include/qPipeCenterlineDialog.h"

// Qt
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>

// Open3D-specific getter implementations
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

void qPipeCenterlineDialog::setRANSACEnabled( bool enabled )
{
	if (m_ransacCheckBox)
	{
		m_ransacCheckBox->setChecked(enabled);
	}
}

void qPipeCenterlineDialog::setRANSACDistanceThreshold( double threshold )
{
	if (m_ransacDistanceSpinBox)
	{
		m_ransacDistanceSpinBox->setValue(threshold);
	}
}

void qPipeCenterlineDialog::setRANSACMaxIterations( int iterations )
{
	if (m_ransacIterationsSpinBox)
	{
		m_ransacIterationsSpinBox->setValue(iterations);
	}
}

void qPipeCenterlineDialog::setMLSREnabled( bool enabled )
{
	if (m_mlsrCheckBox)
	{
		m_mlsrCheckBox->setChecked(enabled);
	}
}

void qPipeCenterlineDialog::setMLSRSearchRadius( double radius )
{
	if (m_mlsrRadiusSpinBox)
	{
		m_mlsrRadiusSpinBox->setValue(radius);
	}
}

void qPipeCenterlineDialog::onRANSACChanged()
{
	updateUI();
}

void qPipeCenterlineDialog::onMLSRChanged()
{
	updateUI();
}