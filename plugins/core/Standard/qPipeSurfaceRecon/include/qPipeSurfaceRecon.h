//##########################################################################
//#                                                                        #
//#                  CLOUDCOMPARE PLUGIN: qPipeSurfaceRecon                #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#                  COPYRIGHT: CloudCompare Developer                     #
//#                                                                        #
//##########################################################################

#ifndef Q_PIPE_SURFACE_RECON_PLUGIN_HEADER
#define Q_PIPE_SURFACE_RECON_PLUGIN_HEADER

#include "ccStdPluginInterface.h"

//! One-click surface reconstruction optimized for pipe point clouds
/** This plugin provides automated surface reconstruction specifically designed
    for pipe-like point clouds. It combines multiple PCL algorithms to achieve
    better results for cylindrical and tubular structures.
**/
class qPipeSurfaceRecon : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qPipeSurfaceRecon" FILE "../info.json" )

public:

	//! Default constructor
	explicit qPipeSurfaceRecon(QObject* parent = nullptr);

	virtual ~qPipeSurfaceRecon() = default;

	//inherited from ccStdPluginInterface
	virtual void onNewSelection(const ccHObject::Container& selectedEntities) override;
	virtual QList<QAction *> getActions() override;

protected:

	//! Slot called when associated action is triggered
	void doAction();

	//! Perform pipe-specific surface reconstruction
	bool performPipeReconstruction(ccPointCloud* cloud);

protected:

	//! Associated action
	QAction* m_action;

};

#endif