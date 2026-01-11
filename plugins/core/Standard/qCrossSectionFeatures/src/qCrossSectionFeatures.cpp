#include "../include/qCrossSectionFeatures.h"
#include "../include/qCrossSectionFeaturesDialog.h"
#include "../include/CrossSectionExtractor.h"

#include <ccMainAppInterface.h>
#include <ccPointCloud.h>
#include <ccHObjectCaster.h>
#include <QMessageBox>
#include <QAction>
#include <QMainWindow> // 确保包含 QMainWindow 定义

qCrossSectionFeatures::qCrossSectionFeatures( QObject* parent )
    : QObject( parent )
    , ccStdPluginInterface( ":/CC/plugin/qCrossSectionFeatures/info.json" )
    , m_action( nullptr )
    , m_dialog( nullptr )
{
    m_action = new QAction( "Tunnel Intelligent Detection", this );
    m_action->setToolTip( "Auto-detect tunnel shape and dimensions" );
    m_action->setIcon( getIcon() );
    connect( m_action, &QAction::triggered, this, &qCrossSectionFeatures::doAction );
}

QIcon qCrossSectionFeatures::getIcon() const { return QIcon( QStringLiteral( ":/CC/plugin/qCrossSectionFeatures/images/qCrossSectionFeatures.png" ) ); }
void qCrossSectionFeatures::onNewSelection( const ccHObject::Container& selectedEntities ) {
    if ( m_action ) m_action->setEnabled( selectedEntities.size() == 1 && selectedEntities[0]->isA( CC_TYPES::POINT_CLOUD ) );
}
QList<QAction*> qCrossSectionFeatures::getActions() { return QList<QAction*>() << m_action; }

void qCrossSectionFeatures::doAction()
{
    if ( !m_app ) return;
    const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
    ccPointCloud* cloud = ccHObjectCaster::ToPointCloud( selectedEntities[0] );
    if ( !cloud ) return;

    // 【修复点 1】强制转换为 QWidget*
    if ( !m_dialog ) 
        m_dialog = new qCrossSectionFeaturesDialog( static_cast<QWidget*>(m_app->getMainWindow()) );
    
    if ( m_dialog->exec() != QDialog::Accepted ) return;

    // 获取参数
    CrossSectionExtractor::Parameters params;
    params.fittingThreshold = m_dialog->getFittingThreshold();
    params.denoiseLevel = m_dialog->getDenoiseLevel();

    // 执行算法
    CrossSectionExtractor extractor( params );
    ccPointCloud* resultCloud = nullptr;
    QString report;

    try {
        extractor.processTunnel( cloud, resultCloud, report );
        
        if ( resultCloud ) {
            m_app->addToDB( resultCloud );
            // 【修复点 2】强制转换为 QWidget*
            QMessageBox::information( static_cast<QWidget*>(m_app->getMainWindow()), "Detection Success", report );
        }
    } catch ( ... ) {
        // 【修复点 3】强制转换为 QWidget*
        QMessageBox::critical( static_cast<QWidget*>(m_app->getMainWindow()), "Error", "An error occurred during processing." );
    }
}
