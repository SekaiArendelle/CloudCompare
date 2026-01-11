#pragma once
#include <QDialog>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>

class qCrossSectionFeaturesDialog : public QDialog
{
    Q_OBJECT

public:
    explicit qCrossSectionFeaturesDialog( QWidget* parent = nullptr );
    ~qCrossSectionFeaturesDialog() override = default;

    // --- 只保留新算法需要的参数 ---
    
    // 1. 拟合误差阈值 (对应 RANSAC 0.05)
    double getFittingThreshold() const;
    
    // 2. 去噪强度 (对应 SOR 1.0)
    double getDenoiseLevel() const;

private:
    void setDefaults();

    QDoubleSpinBox* m_fittingThresholdSpin;
    QDoubleSpinBox* m_denoiseLevelSpin;
};
