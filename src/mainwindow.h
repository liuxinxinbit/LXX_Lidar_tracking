#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <algorithm>
#include <QMainWindow>
#include "vtkRenderWindow.h"
#include "boost/thread/thread.hpp"
#include <QFileInfoList>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <math.h>
#include <vtkSmartPointer.h>
#include <vtkRenderWindowInteractor.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

using namespace std;    
using namespace cv;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


class cloud_object
{
  public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr xcloud;
  pcl::PointXYZ minPt, maxPt;
  vector<double> pccenter;
  long cloud_pointnumber;
  float cloud_radius;
  vector<float> transformation_vector;
  PointT direction;
  vector<PointT> direction_vector;
  vector<int> Converged_vector;
};

namespace Ui {
	class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
	explicit MainWindow(QWidget *parent = 0);
    ~MainWindow() override;
	
protected:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
	PointCloudT::Ptr cloud;
	PointT center;
	PointT center2end;
private:
	Ui::MainWindow *ui;
	
private slots:

    void fileselect_clicked();
	void toolButton_clicked();
	void getfile();
	QStringList getallpcdlist();

	vector<float>  Rectangular2SphericalCoordinateSystem(float x,float y,float z);
	vector<float>  Spherical2RectangularCoordinateSystem(float radius,float theta,float phi);
	float point_length(float x,float y,float z);
	vector<double> getpointcloudcenter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
	cloud_object initial_cloud_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	long *circle_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	bool judge_find(std::vector<int> indices,int value);
	vector<float> getpointcloudbound(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
	vector<cloud_object> pointCloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr PointcloudDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloudheadingcorrect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void cloud_target_screen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_radius_crop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ searchPoint,float radius);
	vector<float> PCL_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
	void target_pointcloud_direction_singlecloud(cloud_object cloud_model,vector<cloud_object> pointcloud_vector,int i);
	void add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool isrun);
	vtkSmartPointer<vtkPolyData> pclcloud2polydata(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void test();
	vector<cloud_object> pointCloud_cluster_simple(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	PointT get_cloud_object_direction(cloud_object co);
	vector<cloud_object> targetcloud_waitfortrack_update(
    vector<cloud_object> targetcloud_waitfortrack,vector<cloud_object>targetcloud_waitfortrack_new);
	pcl::PointCloud<PointT>::Ptr pointcloud_tansform(pcl::PointCloud<PointT>::Ptr cloud, float theta_);
};
#endif

