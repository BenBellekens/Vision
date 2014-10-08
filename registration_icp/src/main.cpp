#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <Eigen/Dense>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PoinT;
using namespace std;

pcl::PointCloud<PoinT>::Ptr source_cloud (new pcl::PointCloud<PoinT>);
pcl::PointCloud<PoinT>::Ptr target_cloud (new pcl::PointCloud<PoinT>);
pcl::PointCloud<PoinT>::Ptr transformed_cloud (new pcl::PointCloud<PoinT>);
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

bool ICPptp = false;
bool ICPpts = false;
bool ICPnl = false;
bool GICP = false;

//----------------------------------------------------------------------------------
//METHODS
//---------------------------------------------------------------------------------- 
void ApplyICPptpRegistration(pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &target_cloud, pcl::PointCloud<PoinT>::Ptr &transformed_cloud){

	pcl::IterativeClosestPoint<PoinT, PoinT> icp;
	icp.setInputSource(source_cloud);
	icp.setInputTarget(target_cloud);

	icp.setTransformationEpsilon (1e-14);
	icp.setMaxCorrespondenceDistance(0.6);
	icp.setMaximumIterations(30);

	pcl::PointCloud<PoinT> Final;
	icp.align(Final);
	std::cout << "icp registration" << std::endl;
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	*transformed_cloud = Final;
}

void ApplyICPptsRegistration(pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &target_cloud, pcl::PointCloud<PoinT>::Ptr &transformed_cloud){
	pcl::PointCloud<pcl::PointNormal>::Ptr source(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*source_cloud, *source);
	pcl::PointCloud<pcl::PointNormal>::Ptr target(new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud(*target_cloud, *target);


	pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
	norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointNormal>::Ptr (new pcl::search::KdTree<pcl::PointNormal>));
	norm_est.setKSearch (10);
	norm_est.setInputCloud (target);
	norm_est.compute (*target);

	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
	typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
	boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
	icp.setTransformationEstimation(point_to_plane);
	icp.setInputSource(source);
	icp.setInputTarget(target);
	icp.setMaximumIterations(30);
	icp.setMaxCorrespondenceDistance(0.6);
	icp.setTransformationEpsilon(1e-14);
	pcl::PointCloud<pcl::PointNormal> Final;
	icp.align(Final);

	std::cout << "icp point to surface registration" << std::endl;
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	pcl::copyPointCloud(Final, *transformed_cloud);
}

void ApplyICPnlRegistration(pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &target_cloud, pcl::PointCloud<PoinT>::Ptr &transformed_cloud){
	pcl::IterativeClosestPointNonLinear<PoinT, PoinT> icpnl;
	icpnl.setInputSource(source_cloud);
	icpnl.setInputTarget(target_cloud);
	icpnl.setMaxCorrespondenceDistance(0.6);
	icpnl.setMaximumIterations(30);
	icpnl.setTransformationEpsilon(1e-14);
	pcl::PointCloud<PoinT> Final;
	icpnl.align(Final);
	std::cout << "icpnl registration" << std::endl;
	std::cout << "has converged:" << icpnl.hasConverged() << " score: " <<	icpnl.getFitnessScore() << std::endl;

	*transformed_cloud = Final;
}

void ApplyGeneralizedICPRegistration(pcl::PointCloud<PoinT>::Ptr &source_cloud, pcl::PointCloud<PoinT>::Ptr &target_cloud, pcl::PointCloud<PoinT>::Ptr &transformed_cloud){
	pcl::GeneralizedIterativeClosestPoint<PoinT, PoinT> gicp;
	gicp.setInputSource(source_cloud);
	gicp.setInputTarget(target_cloud);
	gicp.setMaximumOptimizerIterations(8);
	gicp.setMaxCorrespondenceDistance(0.6);
	gicp.setMaximumIterations(30);
	gicp.setTransformationEpsilon(1e-14);
	pcl::PointCloud<PoinT> Final;
	gicp.align(Final);
	std::cout << "gicp registration" << std::endl;
	std::cout << "has converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
	
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, gicp.getFinalTransformation());
}

//----------------------------------------------------------------------------------
int main (int argc, char** argv)
{

  //----------------------------------------------------------------------------------
  //Read pcd file
  //---------------------------------------------------------------------------------- 
  if (pcl::io::loadPCDFile<PoinT> ("Cosyslab-0.pcd", *source_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file source_cloud.pcd \n");
    return (-1);
  }
  cout << "Loaded " << source_cloud->width * source_cloud->height << " data points "<< endl;

 if (pcl::io::loadPCDFile<PoinT> ("Cosyslab-1.pcd", *target_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file target_test_nonoise.pcd \n");
    return (-1);
  }
  cout << "Loaded " << target_cloud->width * target_cloud->height << " data points "<< endl;

  //----------------------------------------------------------------------------------
  //remove NAN points from the cloud
  //----------------------------------------------------------------------------------
  std::vector<int> indices_src, indices_tgt;
  pcl::removeNaNFromPointCloud(*source_cloud,*source_cloud, indices_src);
  pcl::removeNaNFromPointCloud(*target_cloud,*target_cloud, indices_tgt);

  //----------------------------------------------------------------------------------
  //Reduce number of points
  //----------------------------------------------------------------------------------
  pcl::VoxelGrid<PoinT> grid, grid1;
  grid.setLeafSize (0.01, 0.01, 0.05);
  grid.setInputCloud (source_cloud);
  grid.filter(*source_cloud);
  cout << "source cloud number of point after voxelgrid: " << source_cloud->points.size() << endl;
  
  grid1.setLeafSize (0.01, 0.01, 0.05);
  grid1.setInputCloud (target_cloud);
  grid1.filter(*target_cloud);
  cout << "target cloud number of point after voxelgrid: " << target_cloud->points.size() << endl;

  //----------------------------------------------------------------------------------
  //Make source cloud blue
  //----------------------------------------------------------------------------------
  for (int i = 0; i < source_cloud->points.size(); ++i) {
	source_cloud->points[i].r = 0;
	source_cloud->points[i].g = 0;
	source_cloud->points[i].b = 255;
  }
  //----------------------------------------------------------------------------------
  //Make Target cloud Red
  //----------------------------------------------------------------------------------
  for (int i = 0; i < target_cloud->points.size(); ++i) {
	target_cloud->points[i].r = 255;
	target_cloud->points[i].g = 0;
	target_cloud->points[i].b = 0;
  }
  
  //----------------------------------------------------------------------------------
  //Apply ICP transformation point to point
  //----------------------------------------------------------------------------------	
  if(ICPptp == true){
	ApplyICPptpRegistration(source_cloud, target_cloud, transformed_cloud);
  }

  //----------------------------------------------------------------------------------
  //Apply ICP transformation point to surface
  //----------------------------------------------------------------------------------
  if(ICPpts == true){
	ApplyICPptsRegistration(source_cloud, target_cloud, transformed_cloud);
  }

  //----------------------------------------------------------------------------------
  //Apply ICP non-lineair transformation point to point
  //----------------------------------------------------------------------------------
  if(ICPnl == true){
	ApplyICPnlRegistration(source_cloud, target_cloud, transformed_cloud);
  }

  //----------------------------------------------------------------------------------
  //Apply ICP non-lineair transformation point to point
  //----------------------------------------------------------------------------------
  if(GICP == true){
	ApplyGeneralizedICPRegistration(source_cloud, target_cloud, transformed_cloud);
  }
  //----------------------------------------------------------------------------------
  //Make transformed cloud White
  //----------------------------------------------------------------------------------
  for (int i = 0; i < transformed_cloud->points.size(); ++i) {
	transformed_cloud->points[i].r = 255;
	transformed_cloud->points[i].g = 255;
	transformed_cloud->points[i].b = 255;
  }
  	
  //----------------------------------------------------------------------------------
  //Show cloud in viewer
  //----------------------------------------------------------------------------------
   viewer.showCloud (source_cloud, "source");
   viewer.showCloud (transformed_cloud, "transformed");
   viewer.showCloud (target_cloud, "target");

  while (!viewer.wasStopped ())
  {
	//while keypress "q" is not pressed
  }
  

  return (0);
}
