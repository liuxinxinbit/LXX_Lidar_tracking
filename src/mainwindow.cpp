#include "mainwindow.h"
#include "../ui/ui_mainwindow.h"
#include <qstring.h>
#include <string>
#include <QtWidgets>
#include <qfiledialog.h>
#include <qdir.h>
#include <QVTKWidget.h>

int user_data;
#define Pi 3.14159265
int pool_size=10;
float boat_heading=0;
float boat_velocity=0;
float target_radius_limit = 20;
long pointcloud_count =0;
float speed_limit_per_second=50000/3600;//50kmm
long file_index=0;
vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > cloud_stream_vector;


vector<cloud_object> targetcloud_waitfortrack;

QFileInfoList filelist;
QString choosed_directory;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
	ui->setupUi(this);

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    // The number of points in the cloud
    cloud->points.resize (0);

    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    connect (ui->readdataButton, SIGNAL (clicked()), this, SLOT (getfile()));
    connect (ui->toolButton, SIGNAL (clicked()), this, SLOT (toolButton_clicked ()));
    connect (ui->comboBox, SIGNAL (currentIndexChanged(QString)), this, SLOT (getallpcdlist ()));

   
    viewer->setCameraPosition(0,0,500,0,0,1,0.0,0.1,-200);

    ui->qvtkWidget->update ();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::fileselect_clicked()
{
   QString filename = QFileDialog::getOpenFileName(this,tr("open file"), "/home/liuxinxin/ToolKit/Data_Lidar" ,tr("PCDfile(*.pcd)"));

    cout<<filename.toStdString()<<endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr readcloud (new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename.toStdString(), *readcloud) == -1) //* load the file
//    {
//        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//    }
//    else
//    {
//        long pointsize = readcloud->width * readcloud->height;
//        cout<<readcloud->points.size()<<endl;
//        std::cout << "Loaded "
//                  << pointsize
//                  << " data points from test_pcd.pcd. "
//                  << std::endl;
//     viewer->updatePointCloud (readcloud, "cloud");
    
//     viewer->resetCamera ();
//     ui->qvtkWidget->update ();
//     ui->pcsizelineEdit->setText(QString::fromStdString(to_string(pointsize)) );

//    }
}

void MainWindow::toolButton_clicked()
{
    choosed_directory = QDir::toNativeSeparators(QFileDialog::getExistingDirectory(this,tr("data directory"),"/home/liuxinxin/ToolKit/Data_Lidar"));
    QDir dir(choosed_directory);
    dir.setFilter(QDir::Dirs);
    filelist = dir.entryInfoList();
    for (int i = 2; i < filelist.size(); ++i)
    {
    QFileInfo fileInfo = filelist.at(i);
    if(ui->comboBox->findText(fileInfo.fileName())==-1)
    ui->comboBox->addItem(fileInfo.fileName());
    ui->comboBox->setCurrentIndex(ui->comboBox->findText(fileInfo.fileName()));
    }

}
QStringList MainWindow::getallpcdlist()
{
    QString targetdirector = ui->comboBox->currentText();

    QDir dir(choosed_directory+"/"+targetdirector);
    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    dir.setSorting(QDir::Size | QDir::Reversed);
    QFileInfoList list = dir.entryInfoList();
    QStringList localpcdfilelist;
    for (int i = 0; i < list.size(); ++i)
    {
    QFileInfo fileInfo = list.at(i);
    // qDebug() << qPrintable(QString("%1 %2").arg(fileInfo.size(), 10)
                                                    // .arg(fileInfo.fileName()));

    QString filepath;
    filepath.append(fileInfo.path());
    filepath+="/"+fileInfo.fileName();
    QString formattail =  filepath.mid(filepath.size()-4,-1);
    if(formattail == ".pcd")
    {
        localpcdfilelist.append(filepath); 
    }
    }
    return localpcdfilelist;
}

void MainWindow::getfile()
{
    
    clock_t start,finish;
    double totaltime;
    start=clock();
    for(size_t i=0;i<410;i++) test();
    finish=clock();
    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
    cout<<"\n此程序的运行时间为"<<totaltime<<"秒！"<<endl;

    //     ui->pcsizelineEdit->setText(QString::number(fileindex+1));
}


void MainWindow::test()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>) ;
    stringstream ss;
    ss << "/home/sss/Lxx_Toolkit/lidar_data/201812041646/test" << file_index<<".pcd";
    cout<<ss.str()<<endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (ss.str(), *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read target pcd file \n");
    }    

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    add_cloud(cloud,true);
    
    PointT spherecenter;
    spherecenter.x=0;
    spherecenter.y=0;
    spherecenter.z=0;
    viewer->addSphere(spherecenter,2,0,0,1,"spherecenter");
    PointT linepoint_1;
    linepoint_1.x=-100;
    linepoint_1.y=0;
    linepoint_1.z=0;
    PointT linepoint_2;
    linepoint_2.x=100;
    linepoint_2.y=0;
    linepoint_2.z=0;
    viewer->addLine(linepoint_1,linepoint_2,1,1,0,"linex");
    linepoint_1.x=0;
    linepoint_1.y=-100;
    linepoint_1.z=0;
    linepoint_2.x=0;
    linepoint_2.y=100;
    linepoint_2.z=0;
    viewer->addLine(linepoint_1,linepoint_2,0,1,1,"liney");
    viewer->addPointCloud(cloud, "cloud");

    for(size_t i=0;i<targetcloud_waitfortrack.size();i++)
    {
        if(targetcloud_waitfortrack[i].transformation_vector[4])
        {
            stringstream ss2;
            ss2 << "cloud" << i;
            // viewer->addPointCloud(targetcloud_waitfortrack[i].xcloud, ss2.str());
            // viewer->setPointCloudRenderingProperties (
            //     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss2.str());
            ss2<<"cube";
            viewer->addCube(
            targetcloud_waitfortrack[i].minPt.x,targetcloud_waitfortrack[i].maxPt.x,
            targetcloud_waitfortrack[i].minPt.y,targetcloud_waitfortrack[i].maxPt.y,
            targetcloud_waitfortrack[i].minPt.z,targetcloud_waitfortrack[i].maxPt.z,
            1.0,0,0,ss2.str());
            viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.3,ss2.str());
            linepoint_1.x=targetcloud_waitfortrack[i].pccenter[0];
            linepoint_1.y=targetcloud_waitfortrack[i].pccenter[1];
            linepoint_1.z=targetcloud_waitfortrack[i].pccenter[2];
            PointT direction = get_cloud_object_direction(targetcloud_waitfortrack[i]);
            linepoint_2.x=linepoint_1.x+direction.x*10;
            linepoint_2.y=linepoint_1.y+direction.y*10;
            linepoint_2.z=linepoint_1.z+direction.z*10;
            ss2<<"Arrow";
            viewer->addArrow(linepoint_2,linepoint_1,0,1,0,true,ss2.str());
        }  
    }
    ui->qvtkWidget->update();
    file_index++;
}

PointT MainWindow::get_cloud_object_direction(cloud_object co)
{
    float x=0, y=0, z=0;
    for(size_t i=0;i<co.direction_vector.size();i++)
    {
        x+=co.direction_vector[i].x;
        y+=co.direction_vector[i].y;
        z+=co.direction_vector[i].z;
    }
    PointT direction;
    direction.x=x/co.direction_vector.size();
    direction.y=y/co.direction_vector.size();
    direction.z=z/co.direction_vector.size();
    return direction;
}


vector<float>  MainWindow::Spherical2RectangularCoordinateSystem(float radius,float theta,float phi)
{
    vector<float> scs;
    float x = radius*sin(theta)*cos(phi);
    float y = radius*sin(theta)*sin(phi);
    float z = radius*cos(theta);
    scs.push_back(x);
    scs.push_back(y);
    scs.push_back(z);
    return scs;
}
vector<float>  MainWindow::Rectangular2SphericalCoordinateSystem(float x,float y,float z)
{
    vector<float> scs;
    float r = sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    float theta = acos(z/r);
    float phi = atan2(y,x);
    scs.push_back(r);
    scs.push_back(theta);
    scs.push_back(phi);
    return scs;
}
float  MainWindow::point_length(float x,float y,float z)
{
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

vector<double> MainWindow::getpointcloudcenter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
    vector<double> pccenter;
	double x_min=0;
	double y_min=0;
	double z_min=0;
	for (size_t i = 0; i < cloud->points.size (); i++)
	{
		x_min=x_min+cloud->points[i].x;
        y_min=y_min+cloud->points[i].y;
        z_min=z_min+cloud->points[i].z;
    }
    pccenter.push_back(x_min/cloud->points.size ());
    pccenter.push_back(y_min/cloud->points.size ());
    pccenter.push_back(z_min/cloud->points.size ());
	return pccenter;
}

cloud_object MainWindow::initial_cloud_object(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    cloud_object co;
    co.xcloud = cloud;
    pcl::getMinMax3D (*cloud, co.minPt, co.maxPt);
    co.pccenter = getpointcloudcenter(cloud);
    co.cloud_pointnumber = cloud->points.size();
    co.cloud_radius = max(point_length(co.minPt.x-co.pccenter[0],co.minPt.y-co.pccenter[1],co.minPt.z-co.pccenter[2]),
    point_length(co.maxPt.x-co.pccenter[0],co.maxPt.y-co.pccenter[1],co.maxPt.z-co.pccenter[2]));
    for(size_t i=0;i<5;i++) co.transformation_vector.push_back(0);
    return co;
}

long *MainWindow::circle_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // cout<<"cloud point num ="<<cloud->points.size ()<<endl;
    float angle_resolution=1;
    int angle_num=(int)(360/angle_resolution);
    long *circle_indice=new long[angle_num];
    long circle_indice_distance[angle_num];
    for (size_t i = 0; i < angle_num; i++)
	{
        circle_indice_distance[i]=999999;
        circle_indice[i]=-1;
  }
    for (size_t i = 0; i < cloud->points.size (); i++)
	{
        
    float x=cloud->points[i].x;
    float y=cloud->points[i].y;
		float z=cloud->points[i].z;
		float ddd=point_length(x,y,z);
		int angle = (int)(atan2(y,x)*180/Pi);
		if(angle<0) angle=angle+360;
        
		if(ddd<circle_indice_distance[angle])
		{
      circle_indice_distance[angle]=ddd;
      circle_indice[angle]=i;
		}

    }
    return circle_indice;
}
bool MainWindow::judge_find(std::vector<int> indices,int value)
{
    vector<int>::iterator it;
    it=std::find(indices.begin(),indices.end(),value);
    if (it!=indices.end())
    {
    return true;
    }
    else
    {
    return false;
    }
}
vector<float> MainWindow::getpointcloudbound(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
  pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D (*cloud, minPt, maxPt);
	vector<float> bounds;
    bounds.push_back(minPt.x);
    bounds.push_back(maxPt.x);
    bounds.push_back(minPt.y);
    bounds.push_back(maxPt.y);
    bounds.push_back(minPt.z);
    bounds.push_back(maxPt.z);
    bounds.push_back(cloud->points.size ());
	return bounds;
}
vector<cloud_object> MainWindow::pointCloud_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    long *cloud_circle = circle_sample(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (10.0); // 10m
    // ec.setMinClusterSize (5);
    // ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    vector<cloud_object> screen_targt_cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
    {
        std::vector<int> indices = it->indices;
        bool flag=false;
        for(size_t i =0;i<360;i++)
        {
            if(judge_find(indices,cloud_circle[i]))
            {
                flag=true;
                break;
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        vector<float> bounds=getpointcloudbound(cloud_cluster);
        if(flag && abs(bounds[1]-bounds[0])/2<target_radius_limit && abs(bounds[3]-bounds[2])/2<target_radius_limit && 
        cloud_cluster->points.size()>10 )
        {
            cloud_object co = initial_cloud_object(cloud_cluster);
            screen_targt_cloud.push_back(co);
        }
    }
    return screen_targt_cloud;
}

vector<cloud_object> MainWindow::pointCloud_cluster_simple(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (10.0); // 10m
    // ec.setMinClusterSize (5);
    // ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    vector<cloud_object> screen_targt_cloud;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        std::vector<int> indices = it->indices;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        vector<float> bounds=getpointcloudbound(cloud_cluster);
        
        if(abs(bounds[1]-bounds[0])/2<target_radius_limit && abs(bounds[3]-bounds[2])/2<target_radius_limit &&  cloud_cluster->points.size ()>10)
        {
            cloud_object co = initial_cloud_object(cloud_cluster);
            
            screen_targt_cloud.push_back(co);
        }
    }
    return screen_targt_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::statistical_outlier_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (5);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    return cloud_filtered;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::PointcloudDownsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.5f, 0.5f, 0.5f);
    vg.filter (*cloud_filtered);
    return cloud_filtered;
}
pcl::PointCloud<PointT>::Ptr MainWindow::pointcloud_tansform(pcl::PointCloud<PointT>::Ptr cloud, float theta_)
{
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  float x=0,y=0,z=1;
  transform (0,0)=cos(theta_)+(1-cos(theta_))*x*x;
  transform (0,1)=(1-cos(theta_))*x*y-sin(theta_)*z;
  transform (0,2)=(1-cos(theta_))*x*z+sin(theta_)*y;
  transform (1,0)=(1-cos(theta_))*x*y+sin(theta_)*z;
  transform (1,1)=cos(theta_)+(1-cos(theta_))*y*y;
  transform (1,2)=(1-cos(theta_))*y*z-sin(theta_)*x;
  transform (2,0)=(1-cos(theta_))*x*z-sin(theta_)*y;
  transform (2,1)=(1-cos(theta_))*y*z+sin(theta_)*x;
  transform (2,2)=cos(theta_)+(1-cos(theta_))*z*z;
  pcl::transformPointCloud (*cloud, *cloud, transform);
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::pointcloudheadingcorrect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        float x= cloud->points[i].x;
        float y= cloud->points[i].y;
        float z= cloud->points[i].z;
        vector<float> pcc = Rectangular2SphericalCoordinateSystem(x,y,z);
        pcc[2]=pcc[2]-boat_heading*Pi/180+Pi;
        vector<float> pcc2 = Spherical2RectangularCoordinateSystem(pcc[0],pcc[1],pcc[2]);
        cloud->points[i].x=pcc2[0];
        cloud->points[i].y=pcc2[1];
        cloud->points[i].z=pcc2[2];

    }
    return cloud;
}

vector<cloud_object> MainWindow::targetcloud_waitfortrack_update(
    vector<cloud_object> targetcloud_waitfortrack,vector<cloud_object>targetcloud_waitfortrack_new)
{
    for(size_t i=0;i<targetcloud_waitfortrack_new.size();i++)
    {
        cloud_object targetcloud_new= targetcloud_waitfortrack_new[i];
        cloud_object targetcloud_best;
        float fitscore=0;
        bool flag = false;
        for(size_t j=0;j<targetcloud_waitfortrack.size();j++)
        {
            cloud_object targetcloud_old= targetcloud_waitfortrack[j];
            vector<float> transformvector = PCL_ICP(targetcloud_new.xcloud,targetcloud_old.xcloud);
            float length = point_length(
            targetcloud_old.pccenter[0]-targetcloud_new.pccenter[0],
            targetcloud_old.pccenter[0]-targetcloud_new.pccenter[0],
            targetcloud_old.pccenter[0]-targetcloud_new.pccenter[0]);
            if(transformvector[3]>fitscore && transformvector[4] && length<speed_limit_per_second*2)
            {
                fitscore = transformvector[3];
                targetcloud_best = targetcloud_old;
                flag = true;
            }
        }
        if(flag)
        {
            targetcloud_waitfortrack_new[i].direction_vector  = targetcloud_best.direction_vector;
            targetcloud_waitfortrack_new[i].Converged_vector  = targetcloud_best.Converged_vector;
        }
    }
    return targetcloud_waitfortrack_new;
}
void MainWindow::cloud_target_screen(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  cloud = PointcloudDownsample(cloud);
  cloud = statistical_outlier_removal(cloud);
//   cloud = pointcloudheadingcorrect(cloud);
  if(targetcloud_waitfortrack.size()==0)
  {
      targetcloud_waitfortrack= pointCloud_cluster(cloud );
  }
  else
  {
      targetcloud_waitfortrack = targetcloud_waitfortrack_update(targetcloud_waitfortrack,pointCloud_cluster(cloud ));
  }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr MainWindow::cloud_radius_crop(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ searchPoint,float radius)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    crop_cloud->width = pointIdxRadiusSearch.size ();
    crop_cloud->height = 1;
    crop_cloud->points.resize (crop_cloud->width * crop_cloud->height);
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
      crop_cloud->points[i]=cloud->points[ pointIdxRadiusSearch[i] ];
    }
  }
  return crop_cloud;
}
vector<float> MainWindow::PCL_ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (100);
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    // pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 Transformation_Matrix;
    //~ std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                 //~ icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f Transformation_Matrix=icp.getFinalTransformation();
    // std::cout << Transformation_Matrix(0,3)<< std::endl;
    vector<float> translation_matrix;
    translation_matrix.push_back(Transformation_Matrix(0,3));
    translation_matrix.push_back(Transformation_Matrix(1,3));
    translation_matrix.push_back(Transformation_Matrix(2,3));
	translation_matrix.push_back(icp.getFitnessScore());
    translation_matrix.push_back(icp.hasConverged());
	return translation_matrix;
}

void MainWindow::target_pointcloud_direction_singlecloud(cloud_object cloud_model,vector<cloud_object> pointcloud_vector,int i)
{
    cloud_object result_cloud;
    vector<float> translation_matrix;
    float maxcore=0;
    if(pointcloud_vector.size()==1)
    {
        translation_matrix = PCL_ICP(cloud_model.xcloud,pointcloud_vector[0].xcloud);

        targetcloud_waitfortrack[i].xcloud = pointcloud_vector[0].xcloud;
        targetcloud_waitfortrack[i].minPt = pointcloud_vector[0].minPt;
        targetcloud_waitfortrack[i].maxPt = pointcloud_vector[0].maxPt;
        targetcloud_waitfortrack[i].pccenter = pointcloud_vector[0].pccenter;
        targetcloud_waitfortrack[i].cloud_pointnumber = pointcloud_vector[0].cloud_pointnumber;
        targetcloud_waitfortrack[i].cloud_radius = pointcloud_vector[0].cloud_radius;
        targetcloud_waitfortrack[i].transformation_vector = translation_matrix;
        targetcloud_waitfortrack[i].direction.x = pointcloud_vector[0].pccenter[0]-cloud_model.pccenter[0];
        targetcloud_waitfortrack[i].direction.y = pointcloud_vector[0].pccenter[1]-cloud_model.pccenter[1];
        targetcloud_waitfortrack[i].direction.z = pointcloud_vector[0].pccenter[2]-cloud_model.pccenter[2];
        
        targetcloud_waitfortrack[i].direction_vector.push_back(targetcloud_waitfortrack[i].direction);
        if(targetcloud_waitfortrack[i].direction_vector.size()>10) 
        targetcloud_waitfortrack[i].direction_vector.erase(targetcloud_waitfortrack[i].direction_vector.begin());
        
        targetcloud_waitfortrack[i].Converged_vector.push_back(translation_matrix[4]);
        if(targetcloud_waitfortrack[i].Converged_vector.size()>10)
        targetcloud_waitfortrack[i].Converged_vector.erase(targetcloud_waitfortrack[i].Converged_vector.begin());
    }
    else
    {
        for(size_t target_index=0;target_index<pointcloud_vector.size();target_index++)
        {
            if(translation_matrix.size()==0)
            {
                translation_matrix = PCL_ICP(cloud_model.xcloud,pointcloud_vector[target_index].xcloud);
                targetcloud_waitfortrack[i].xcloud = pointcloud_vector[target_index].xcloud;
                targetcloud_waitfortrack[i].minPt = pointcloud_vector[target_index].minPt;
                targetcloud_waitfortrack[i].maxPt = pointcloud_vector[target_index].maxPt;
                targetcloud_waitfortrack[i].pccenter = pointcloud_vector[target_index].pccenter;
                targetcloud_waitfortrack[i].cloud_pointnumber = pointcloud_vector[target_index].cloud_pointnumber;
                targetcloud_waitfortrack[i].cloud_radius = pointcloud_vector[target_index].cloud_radius;
                targetcloud_waitfortrack[i].transformation_vector = translation_matrix;
                targetcloud_waitfortrack[i].direction.x = pointcloud_vector[target_index].pccenter[0]-cloud_model.pccenter[0];
                targetcloud_waitfortrack[i].direction.y = pointcloud_vector[target_index].pccenter[1]-cloud_model.pccenter[1];
                targetcloud_waitfortrack[i].direction.z = pointcloud_vector[target_index].pccenter[2]-cloud_model.pccenter[2];
        
                targetcloud_waitfortrack[i].direction_vector.push_back(targetcloud_waitfortrack[i].direction);
                if(targetcloud_waitfortrack[i].direction_vector.size()>10) 
                targetcloud_waitfortrack[i].direction_vector.erase(targetcloud_waitfortrack[i].direction_vector.begin());
        
                targetcloud_waitfortrack[i].Converged_vector.push_back(translation_matrix[4]);
                if(targetcloud_waitfortrack[i].Converged_vector.size()>10)
                targetcloud_waitfortrack[i].Converged_vector.erase(targetcloud_waitfortrack[i].Converged_vector.begin());
            }
            else
            {
                vector<float> translation_matrix_l = PCL_ICP(cloud_model.xcloud,pointcloud_vector[target_index].xcloud);
                if(translation_matrix_l[3]>maxcore)
                {
                    maxcore = translation_matrix[3];
                    translation_matrix = translation_matrix_l;
                    targetcloud_waitfortrack[i].xcloud = pointcloud_vector[target_index].xcloud;
                    targetcloud_waitfortrack[i].minPt = pointcloud_vector[target_index].minPt;
                    targetcloud_waitfortrack[i].maxPt = pointcloud_vector[target_index].maxPt;
                    targetcloud_waitfortrack[i].pccenter = pointcloud_vector[target_index].pccenter;
                    targetcloud_waitfortrack[i].cloud_pointnumber = pointcloud_vector[target_index].cloud_pointnumber;
                    targetcloud_waitfortrack[i].cloud_radius = pointcloud_vector[target_index].cloud_radius;
                    targetcloud_waitfortrack[i].transformation_vector = translation_matrix;
                    targetcloud_waitfortrack[i].direction.x = pointcloud_vector[target_index].pccenter[0]-cloud_model.pccenter[0];
                    targetcloud_waitfortrack[i].direction.y = pointcloud_vector[target_index].pccenter[1]-cloud_model.pccenter[1];
                    targetcloud_waitfortrack[i].direction.z = pointcloud_vector[target_index].pccenter[2]-cloud_model.pccenter[2];
        
                    targetcloud_waitfortrack[i].direction_vector.push_back(targetcloud_waitfortrack[i].direction);
                    if(targetcloud_waitfortrack[i].direction_vector.size()>10) 
                    targetcloud_waitfortrack[i].direction_vector.erase(targetcloud_waitfortrack[i].direction_vector.begin());
        
                    targetcloud_waitfortrack[i].Converged_vector.push_back(translation_matrix[4]);
                    if(targetcloud_waitfortrack[i].Converged_vector.size()>10)
                    targetcloud_waitfortrack[i].Converged_vector.erase(targetcloud_waitfortrack[i].Converged_vector.begin());
                }
            }
        }
    }
}

void MainWindow::add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool isrun)
{
    // cloud = pointcloud_tansform(cloud, Pi-boat_heading*Pi/180);
    if(file_index%pool_size==0) cloud_target_screen(cloud);
    cloud_stream_vector.push_back(cloud);
    if(cloud_stream_vector.size()>pool_size)
    {
        if(isrun)
        {
            for(size_t i=0;i<targetcloud_waitfortrack.size();i++)
            {
            cloud_object target = targetcloud_waitfortrack[i];
            pcl::PointXYZ searchPoint;
            searchPoint.x = target.pccenter[0];
            searchPoint.y = target.pccenter[1];
            searchPoint.z = target.pccenter[2];
            pcl::PointCloud<pcl::PointXYZ>::Ptr crop_cloud;
            if(cloud->points.size()>0)
            crop_cloud = cloud_radius_crop(cloud,searchPoint,speed_limit_per_second*pool_size/10+target.cloud_radius);
            if(crop_cloud->points.size())
            {
                vector< cloud_object> pointcloud_screen = pointCloud_cluster_simple(crop_cloud );
                target_pointcloud_direction_singlecloud(target,pointcloud_screen,i);
            }           
            }
        } 
        cloud_stream_vector.erase(cloud_stream_vector.begin());
    }
}


vtkSmartPointer<vtkPolyData> MainWindow::pclcloud2polydata(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    float p[3] = {0.0, 0.0, 0.0};
    for(size_t i=0;i<cloud->points.size();i++)
    {
        p[0]=cloud->points[i].x;
        p[1]=cloud->points[i].y;
        p[2]=cloud->points[i].z;
        points->InsertNextPoint(p);
    }
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    return polydata;
}



