// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// ##########################################################################

#include "../include/qPipeCenterline.h"

// Qt
#include <QAction>
#include <QIcon>
#include <QMainWindow>
#include <QMessageBox>
#include <QString>

// qCC
#include "ccMainAppInterface.h"

qPipeCenterline::qPipeCenterline(QObject* parent)
    : QObject(parent)
    , ccStdPluginInterface(":/CC/plugin/qPipeCenterline/info.json")
    , m_action(nullptr)
{
	// Create action
	m_action = new QAction(getName(), this);
	m_action->setToolTip(getDescription());
	m_action->setIcon(getIcon());

	connect(m_action, &QAction::triggered, this, &qPipeCenterline::doAction);
}

QIcon qPipeCenterline::getIcon() const
{
	return QIcon(QStringLiteral(":/CC/plugin/qPipeCenterline/images/qPipeCenterline.png"));
}

void qPipeCenterline::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action == nullptr)
	{
		return;
	}

	// 保持插件入口可用，即便没有具体处理逻辑
	m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction*> qPipeCenterline::getActions()
{
	return QList<QAction*>() << m_action;
}

void qPipeCenterline::doAction()
{
	// 功能已移除，仅保留插件加载框架
	QMessageBox::information(m_app ? m_app->getMainWindow() : nullptr,
	                         "qPipeCenterline",
	                         "to be implemented");
}