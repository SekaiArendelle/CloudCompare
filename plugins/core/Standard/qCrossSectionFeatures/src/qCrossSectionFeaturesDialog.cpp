#include "../include/qCrossSectionFeaturesDialog.h"
#include <QDialogButtonBox>
#include <QGridLayout>

qCrossSectionFeaturesDialog::qCrossSectionFeaturesDialog( QWidget* parent )
    : QDialog( parent )
{
    setWindowTitle( "Tunnel Intelligence Parameters" );
    resize( 300, 200 );

    QVBoxLayout* mainLayout = new QVBoxLayout( this );
    QGroupBox* group = new QGroupBox( "Algorithm Settings", this );
    QGridLayout* layout = new QGridLayout( group );

    // 参数 1: 拟合误差 (原代码 0.05)
    layout->addWidget( new QLabel( "Fitting Threshold (m):", group ), 0, 0 );
    m_fittingThresholdSpin = new QDoubleSpinBox( group );
    m_fittingThresholdSpin->setRange( 0.001, 1.0 ); // 0.0 - 1.0 范围
    m_fittingThresholdSpin->setDecimals( 3 );
    m_fittingThresholdSpin->setSingleStep( 0.01 );
    m_fittingThresholdSpin->setValue( 0.05 ); // 默认值
    m_fittingThresholdSpin->setToolTip( "Max distance for RANSAC inliers (default: 0.05m)" );
    layout->addWidget( m_fittingThresholdSpin, 0, 1 );

    // 参数 2: 去噪强度 (原代码 1.0)
    layout->addWidget( new QLabel( "Denoise Sigma (Thresh):", group ), 1, 0 );
    m_denoiseLevelSpin = new QDoubleSpinBox( group );
    m_denoiseLevelSpin->setRange( 0.1, 5.0 );
    m_denoiseLevelSpin->setDecimals( 1 );
    m_denoiseLevelSpin->setSingleStep( 0.1 );
    m_denoiseLevelSpin->setValue( 1.0 ); // 默认值
    m_denoiseLevelSpin->setToolTip( "Stddev multiplier for SOR filter (default: 1.0)" );
    layout->addWidget( m_denoiseLevelSpin, 1, 1 );

    mainLayout->addWidget( group );

    // 按钮
    QDialogButtonBox* buttons = new QDialogButtonBox( QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this );
    connect( buttons, &QDialogButtonBox::accepted, this, &QDialog::accept );
    connect( buttons, &QDialogButtonBox::rejected, this, &QDialog::reject );
    mainLayout->addWidget( buttons );
}

double qCrossSectionFeaturesDialog::getFittingThreshold() const { return m_fittingThresholdSpin->value(); }
double qCrossSectionFeaturesDialog::getDenoiseLevel() const { return m_denoiseLevelSpin->value(); }
void qCrossSectionFeaturesDialog::setDefaults() { m_fittingThresholdSpin->setValue(0.05); m_denoiseLevelSpin->setValue(1.0); }
