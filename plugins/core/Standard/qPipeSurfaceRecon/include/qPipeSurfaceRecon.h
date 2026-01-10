#ifndef Q_PIPE_SURFACE_RECON_H
#define Q_PIPE_SURFACE_RECON_H

#include <ccStdPluginInterface.h>
#include <QObject>
#include <QAction>
#include <QList>

class qPipeSurfaceRecon : public QObject, public ccStdPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(ccPluginInterface)

    // 【关键修改 2】 确保 FILE 后面只有文件名，不需要路径（因为它们在同一目录下）
    Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.interface" FILE "../info.json")

public:
    explicit qPipeSurfaceRecon(QObject* parent = nullptr);
    ~qPipeSurfaceRecon() override = default;

    void onNewSelection(const ccHObject::Container& selectedEntities) override;
    QList<QAction *> getActions() override;

public slots:
    void doAction();

private:
    QAction* m_action = nullptr;
};

#endif
