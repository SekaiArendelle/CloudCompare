#pragma once

// ##########################################################################
// #                                                                        #
// #                CLOUDCOMPARE PLUGIN: qPipeCenterline                    #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #                      COPYRIGHT: CloudCompare Developer                 #
// #                                                                        #
// ##########################################################################

// Qt
#include <QObject>

// qCC
#include "ccStdPluginInterface.h"

// CCCoreLib
#include <ccPointCloud.h>
#include <ccPolyline.h>

// System
#include <vector>

class QAction;
class ccHObject;

class qPipeCenterlineDialog;

//! Pipe centerline extraction plugin
class qPipeCenterline : public QObject
    , public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qPipeCenterline" FILE "../info.json")

  public:
	//! Default constructor
	explicit qPipeCenterline(QObject* parent = nullptr);

	//! Destructor
	~qPipeCenterline() override = default;

	// Inherited from ccPluginInterface
	QString getName() const override
	{
		return "Pipe Centerline Extraction";
	}
	QString getDescription() const override
	{
		return "Extract and render centerlines from pipe point clouds";
	}
	QIcon getIcon() const override;

	// Inherited from ccStdPluginInterface
	void            onNewSelection(const ccHObject::Container& selectedEntities) override;
	QList<QAction*> getActions() override;

  protected:
	//! Slot called when the action is triggered
	void doAction();

	//! Slot to handle the extraction process
	void extractCenterline();

  private:
	//! Initialize the plugin

	//! Extract pipe centerline using skeletonization
	bool extractPipeCenterline(ccPointCloud* cloud, std::vector<ccPolyline*>& centerlines);

	//! Handle pipe branches and bifurcations
	bool handleBranches(const std::vector<ccPolyline*>& centerlines,
	                    std::vector<ccPolyline*>&       finalCenterlines);

	//! Render centerlines
	void renderCenterlines(const std::vector<ccPolyline*>& centerlines);

	//! Check if selected entity is a valid point cloud for pipe processing
	bool isValidPipeCloud(ccHObject* entity);

	// Actions
	QAction* m_action;

	// UI Dialog
	qPipeCenterlineDialog* m_dialog;

	// Currently selected point cloud
	ccPointCloud* m_selectedCloud;
};