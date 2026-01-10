#pragma once

// Qt
#include <QObject>

// CloudCompare
#include <ccStdPluginInterface.h>

class QAction;
class qCrossSectionFeaturesDialog;

class qCrossSectionFeatures : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
    // 注意：这里必须和你的 info.json 以及 CMakeLists.txt 里的定义匹配
    Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qCrossSectionFeatures" FILE "../info.json" )

public:
    explicit qCrossSectionFeatures( QObject* parent = nullptr );
    ~qCrossSectionFeatures() override = default;

    // 继承自 ccPluginInterface
    QString getName() const override { return "Cross Section Features"; }
    QString getDescription() const override { return "Tunnel Intelligent Detection System"; }
    QIcon getIcon() const override;

    // 继承自 ccStdPluginInterface
    void onNewSelection( const ccHObject::Container& selectedEntities ) override;
    QList<QAction *> getActions() override;

public slots:
    // 核心执行函数
    void doAction();

private:
    // 菜单动作
    QAction* m_action;

    // 参数对话框 (复用，避免重复创建)
    qCrossSectionFeaturesDialog* m_dialog;
};
