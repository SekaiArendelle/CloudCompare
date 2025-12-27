#pragma once

// ##########################################################################
// #                                                                        #
// #            CLOUDCOMPARE PLUGIN: qCrossSectionFeatures                  #
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

class qCrossSectionFeaturesDialog;

//! Cross section feature extraction plugin
class qCrossSectionFeatures : public QObject, public ccStdPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccPluginInterface ccStdPluginInterface )
	Q_PLUGIN_METADATA( IID "cccorp.cloudcompare.plugin.qCrossSectionFeatures" FILE "../info.json" )

public:
	//! Default constructor
	explicit qCrossSectionFeatures( QObject* parent = nullptr );
	
	//! Destructor
	~qCrossSectionFeatures() override = default;

	// Inherited from ccPluginInterface
	QString getName() const override { return "Cross Section Features"; }
	QString getDescription() const override { return "Extract feature point coordinates from point cloud cross sections"; }
	QIcon getIcon() const override;
	
	// Inherited from ccStdPluginInterface
	void onNewSelection( const ccHObject::Container& selectedEntities ) override;
	QList<QAction *> getActions() override;
	
protected:
	//! Slot called when the action is triggered
	void doAction();
	
	//! Slot to handle the extraction process
	void extractFeatures();
	
private:
	//! Check if selected entity is a valid point cloud for processing
	bool isValidCrossSectionCloud( ccHObject* entity );
	
	//! Create polyline from point sequence
	ccPolyline* createPolyline( const std::vector<CCVector3>& points, const QString& name );
	
	// Actions
	QAction* m_action;
	
	// UI Dialog
	qCrossSectionFeaturesDialog* m_dialog;
	
	// Currently selected point cloud
	ccPointCloud* m_selectedCloud;
};