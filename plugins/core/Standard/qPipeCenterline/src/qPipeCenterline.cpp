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
#include "ccHObjectCaster.h"
#include "ccMainAppInterface.h"
#include "ccPointCloud.h"
#include "ccPolyline.h"
#include "ccLog.h"
#include "ccColorTypes.h"
#include "ccBBox.h"

// PCL & algorithm
#include "../include/centerline_extractor.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

	// Keep plugin entry enabled even without specific processing logic
	m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction*> qPipeCenterline::getActions()
{
	return QList<QAction*>() << m_action;
}

void qPipeCenterline::doAction()
{
	if (!m_app)
	{
		return;
	}

	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	if (selectedEntities.empty())
	{
		ccLog::Warning("Please select a point cloud object");
		return;
	}

	ccPointCloud* ccCloud = ccHObjectCaster::ToPointCloud(selectedEntities.front());
	if (!ccCloud)
	{
		ccLog::Warning("Only point cloud entities are supported");
		return;
	}

	if (ccCloud->size() < 10)
	{
		ccLog::Warning("Not enough points to extract a centerline");
		return;
	}

	// Estimate a suitable slice resolution: 0.5% of the bounding-box diagonal length
	ccBBox bbox = ccCloud->getOwnBB();
	double diag = bbox.isValid() ? bbox.getDiagNormd() : 0.0;
	float sliceResolution = static_cast<float>(diag * 0.005);
	if (sliceResolution <= 0.f)
	{
		sliceResolution = 0.1f;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
	pclCloud->resize(ccCloud->size());

	for (unsigned i = 0; i < ccCloud->size(); ++i)
	{
		const CCVector3* P = ccCloud->getPoint(i);
		pclCloud->at(i).x = static_cast<float>(P->x);
		pclCloud->at(i).y = static_cast<float>(P->y);
		pclCloud->at(i).z = static_cast<float>(P->z);
	}

	CenterlineExtractor extractor;
	extractor.setPointCloud(pclCloud);
	extractor.extract(sliceResolution);

    const auto& tracks = extractor.getCenterlineTracks();
    if (tracks.empty())
	{
		ccLog::Warning("Not enough centerline points; extraction failed");
		return;
	}

    ccHObject* group = new ccHObject(ccCloud->getName() + " - Centerlines");
    unsigned created = 0;
    for (std::size_t ti = 0; ti < tracks.size(); ++ti)
    {
        const auto& centerline = tracks[ti];
        if (centerline.size() < 2)
        {
            continue;
        }

        ccPointCloud* vertices = new ccPointCloud(QString("CenterlineVertices_%1").arg(ti + 1));
        if (!vertices->reserve(static_cast<unsigned>(centerline.size())))
        {
            ccLog::Error("Not enough memory to create the centerline");
            delete vertices;
            continue;
        }

        for (const auto& p : centerline)
        {
            vertices->addPoint(CCVector3(p.x(), p.y(), p.z()));
        }
        vertices->setEnabled(false);

        ccPolyline* polyline = new ccPolyline(vertices);
        if (!polyline->reserve(vertices->size()))
        {
            ccLog::Error("Not enough memory to create the centerline polyline");
            delete polyline;
            delete vertices;
            continue;
        }

        polyline->addPointIndex(0, vertices->size());
        polyline->setClosed(false);
        polyline->setName(ccCloud->getName() + QString(" - Centerline #%1").arg(ti + 1));
        polyline->setColor(ccColor::red);
        polyline->showColors(true);
        polyline->setWidth(2);
        polyline->addChild(vertices);
        polyline->copyGlobalShiftAndScale(*ccCloud);
        polyline->setDisplay_recursive(ccCloud->getDisplay());

        group->addChild(polyline);
        ++created;
    }

    if (created > 0)
    {
        m_app->addToDB(group);
        m_app->redrawAll();
        ccLog::Print(QStringLiteral("Centerline extraction completed with %1 branches; slice resolution about %2").arg(created).arg(sliceResolution));
    }
    else
    {
        delete group;
        ccLog::Warning("Centerline extraction produced no valid branches");
    }
}