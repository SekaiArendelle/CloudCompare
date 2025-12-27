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

#include "qPipeSurfaceRecon.h"

//Qt
#include <QProgressDialog>
#include <QApplication>
#include <QtConcurrentRun>
#include <QFuture>
#include <QMessageBox>
#include <QMainWindow>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/kdtree.h>

//qCC_db
#include <ccPointCloud.h>
#include <ccMesh.h>
#include <ccProgressDialog.h>
#include <ccLog.h>

//system
#if defined(CC_WINDOWS)
#include "Windows.h"
#else
#include <unistd.h>
#endif

qPipeSurfaceRecon::qPipeSurfaceRecon(QObject* parent/*=nullptr*/)
	: QObject(parent)
	, ccStdPluginInterface(":/CC/plugin/qPipeSurfaceRecon/info.json")
	, m_action(nullptr)
{
}

void qPipeSurfaceRecon::onNewSelection(const ccHObject::Container& selectedEntities)
{
	if (m_action)
	{
		// Enable action only when exactly one point cloud is selected
		bool enable = (selectedEntities.size() == 1 && selectedEntities[0]->isA(CC_TYPES::POINT_CLOUD));
		m_action->setEnabled(enable);
	}
}

QList<QAction *> qPipeSurfaceRecon::getActions()
{
	// Default action
	if (!m_action)
	{
		m_action = new QAction(getName(), this);
		m_action->setToolTip(getDescription());
		m_action->setIcon(getIcon());
		// Connect signal
		connect(m_action, &QAction::triggered, this, &qPipeSurfaceRecon::doAction);
	}

	return QList<QAction *>{ m_action };
}

// Helper function to convert CC point cloud to PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr ccToPCL(ccPointCloud* ccCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	unsigned pointCount = ccCloud->size();
	pclCloud->width = pointCount;
	pclCloud->height = 1;
	pclCloud->is_dense = false;
	pclCloud->points.resize(pointCount);
	
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* P = ccCloud->getPoint(i);
		pclCloud->points[i].x = static_cast<float>(P->x);
		pclCloud->points[i].y = static_cast<float>(P->y);
		pclCloud->points[i].z = static_cast<float>(P->z);
	}
	
	return pclCloud;
}

// Helper function to convert PCL mesh to CC mesh
ccMesh* pclToCC(const pcl::PolygonMesh& pclMesh, const QString& name)
{
	// Convert PCL mesh vertices to CC point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclVertices(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(pclMesh.cloud, *pclVertices);
	
	ccPointCloud* ccVertices = new ccPointCloud(name + " vertices");
	ccVertices->reserve(pclVertices->size());
	
	for (const auto& point : pclVertices->points)
	{
		CCVector3 P(point.x, point.y, point.z);
		ccVertices->addPoint(P);
	}
	
	ccVertices->shrinkToFit();
	
	// Create CC mesh
	ccMesh* ccMeshObj = new ccMesh(ccVertices);
	ccMeshObj->reserve(pclMesh.polygons.size());
	
	// Add triangles
	for (const auto& polygon : pclMesh.polygons)
	{
		if (polygon.vertices.size() == 3)
		{
			ccMeshObj->addTriangle(polygon.vertices[0], polygon.vertices[1], polygon.vertices[2]);
		}
	}
	
	ccVertices->shrinkToFit();
	ccMeshObj->shrinkToFit();
	ccMeshObj->setName(name);
	
	return ccMeshObj;
}

// Thread-safe reconstruction function
static ccPointCloud* s_inputCloud = nullptr;
static ccMesh* s_outputMesh = nullptr;
static bool s_reconstructionSuccess = false;

bool performReconstructionThread()
{
	if (!s_inputCloud)
	{
		return false;
	}
	
	try
	{
		// Convert CC cloud to PCL format
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = ccToPCL(s_inputCloud);
		
		// Check if we have enough points
		if (pclCloud->empty() || pclCloud->size() < 10)
		{
			ccLog::Error("Not enough points for reconstruction");
			return false;
		}
		
		ccLog::Print(QString("[PipeRecon] Starting with %1 points").arg(pclCloud->size()));
		
		// Step 1: Remove statistical outliers (common in pipe scans)
		ccLog::Print("[PipeRecon] Removing statistical outliers...");
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(pclCloud);
		sor.setMeanK(30); // Reduced for better surface preservation
		sor.setStddevMulThresh(1.5); // More conservative filtering
		sor.filter(*pclCloud);
		
		ccLog::Print(QString("[PipeRecon] After outlier removal: %1 points").arg(pclCloud->size()));
		
		if (pclCloud->empty() || pclCloud->size() < 10)
		{
			ccLog::Error("Too few points after outlier removal");
			return false;
		}
		
		// Step 2: Surface-aware MLS smoothing
		ccLog::Print("[PipeRecon] Surface-aware MLS smoothing...");
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSmoothed(new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr treeMLS(new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
		mls.setInputCloud(pclCloud);
		mls.setPolynomialOrder(2);
		mls.setSearchMethod(treeMLS);
		mls.setSearchRadius(0.006); // Reduced radius for better surface preservation
		mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::NONE);
		mls.process(*cloudSmoothed);
		
		ccLog::Print(QString("[PipeRecon] After MLS smoothing: %1 points").arg(cloudSmoothed->size()));
		
		if (cloudSmoothed->empty() || cloudSmoothed->size() < 10)
		{
			ccLog::Error("Too few points after MLS smoothing");
			return false;
		}
		
		// Step 3: High-quality normal estimation for surface reconstruction
		ccLog::Print("[PipeRecon] Estimating high-quality normals...");
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloudSmoothed);
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		ne.setKSearch(40); // Increased for better surface normal estimation
		ne.compute(*normals);
		
		// Ensure consistent normal orientation for surface reconstruction
		for (size_t i = 0; i < normals->size(); ++i)
		{
			// Make normals point outward (positive direction)
			if (normals->points[i].normal_x + normals->points[i].normal_y + normals->points[i].normal_z < 0)
			{
				normals->points[i].normal_x *= -1;
				normals->points[i].normal_y *= -1;
				normals->points[i].normal_z *= -1;
			}
		}
		
		// Step 4: Create point cloud with normals
		pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloudSmoothed, *normals, *cloudWithNormals);
		
		ccLog::Print(QString("[PipeRecon] Cloud with normals: %1 points").arg(cloudWithNormals->size()));
		
		// Step 5: Advanced surface reconstruction - Poisson with anti-fragmentation
		ccLog::Print("[PipeRecon] Advanced Poisson surface reconstruction...");
		
		pcl::Poisson<pcl::PointNormal> poisson;
		
		// Carefully tuned parameters for surface reconstruction
		poisson.setDepth(8); // Good balance of detail and smoothness
		poisson.setMinDepth(5);
		poisson.setPointWeight(3.0); // Higher weight for better surface definition
		poisson.setScale(1.1);
		poisson.setSolverDivide(8);
		poisson.setIsoDivide(8);
		poisson.setSamplesPerNode(1.0);
		poisson.setConfidence(false);
		poisson.setManifold(true);
		poisson.setOutputPolygons(true);
		
		pcl::PolygonMesh pclMesh;
		poisson.setInputCloud(cloudWithNormals);
		poisson.reconstruct(pclMesh);
		
		ccLog::Print(QString("[PipeRecon] Poisson reconstruction: %1 polygons").arg(pclMesh.polygons.size()));
		
		// Step 6: Surface quality validation and post-processing
		if (pclMesh.polygons.size() > 0)
		{
			// Convert result back to CC format
			s_outputMesh = pclToCC(pclMesh, QString("PipeRecon[%1]").arg(s_inputCloud->getName()));
			
			if (s_outputMesh && s_outputMesh->size() > 0)
			{
				ccLog::Print(QString("[PipeRecon] Reconstruction successful: %1 triangles, %2 vertices")
					.arg(s_outputMesh->size())
					.arg(s_outputMesh->getAssociatedCloud()->size()));
				
				// Validate surface quality
				int triangles = s_outputMesh->size();
				int vertices = s_outputMesh->getAssociatedCloud()->size();
				
				// Check if we have a reasonable mesh
				if (triangles < 100)
				{
					ccLog::Warning("[PipeRecon] Warning: Very few triangles generated, surface may be incomplete");
				}
				else if (triangles > vertices * 10)
				{
					ccLog::Warning("[PipeRecon] Warning: High triangle density may indicate surface artifacts");
				}
				
				return true;
			}
		}
		
		ccLog::Error("Reconstruction failed or produced empty mesh");
		return false;
	}
	catch (const std::exception& e)
	{
		ccLog::Error(QString("Reconstruction error: %1").arg(e.what()));
		return false;
	}
	catch (...)
	{
		ccLog::Error("Unknown error during reconstruction");
		return false;
	}
}

void qPipeSurfaceRecon::doAction()
{
	assert(m_app);
	if (!m_app)
	{
		return;
	}
	
	// Check selection
	if (!m_app->haveOneSelection())
	{
		m_app->dispToConsole("Please select exactly one point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	const ccHObject::Container& selectedEntities = m_app->getSelectedEntities();
	ccHObject* ent = selectedEntities[0];
	
	if (!ent->isA(CC_TYPES::POINT_CLOUD))
	{
		m_app->dispToConsole("Please select a point cloud!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	ccPointCloud* pc = static_cast<ccPointCloud*>(ent);
	
	// Check minimum point count
	if (pc->size() < 100)
	{
		m_app->dispToConsole("Point cloud must have at least 100 points!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
		return;
	}
	
	// Show progress dialog
	QWidget* parentWidget = m_app->getMainWindow() ? static_cast<QWidget*>(m_app->getMainWindow()) : nullptr;
	QProgressDialog pDlg(QString("Pipe Surface Reconstruction"), QString("Cancel"), 0, 0, parentWidget);
	pDlg.setWindowTitle("Pipe Reconstruction");
	pDlg.setCancelButton(nullptr); // No cancel for now
	pDlg.setMinimumDuration(0);
	pDlg.show();
	QApplication::processEvents();
	
	// Store input and reset output
	s_inputCloud = pc;
	s_outputMesh = nullptr;
	s_reconstructionSuccess = false;
	
	// Run reconstruction in separate thread
	QFuture<bool> future = QtConcurrent::run(performReconstructionThread);
	
	// Wait with progress updates
	while (!future.isFinished())
	{
	#if defined(CC_WINDOWS)
		::Sleep(500);
	#else
		usleep(500 * 1000);
	#endif
		
		pDlg.setValue(pDlg.value() + 1);
		QApplication::processEvents();
	}
	
	s_reconstructionSuccess = future.result();
	pDlg.hide();
	
	// Process results
	if (s_reconstructionSuccess && s_outputMesh)
	{
		// Add mesh to database
		m_app->addToDB(s_outputMesh);
		
		// Select the new mesh
		m_app->setSelectedInDB(ent, false);
		m_app->setSelectedInDB(s_outputMesh, true);
		
		// Update UI
		m_app->updateUI();
		m_app->refreshAll();
		
		m_app->dispToConsole("Pipe surface reconstruction completed successfully!", ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}
	else
	{
		m_app->dispToConsole("Pipe surface reconstruction failed!", ccMainAppInterface::ERR_CONSOLE_MESSAGE);
	}
	
	// Cleanup
	s_inputCloud = nullptr;
	if (!s_reconstructionSuccess && s_outputMesh)
	{
		delete s_outputMesh;
	}
	s_outputMesh = nullptr;
}