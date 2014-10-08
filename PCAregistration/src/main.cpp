#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <Eigen/Dense>
#include <pcl/visualization/image_viewer.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGB PoinT;
using namespace std;

pcl::PointCloud<PoinT>::Ptr source_cloud (new pcl::PointCloud<PoinT>);
pcl::PointCloud<PoinT>::Ptr target_cloud (new pcl::PointCloud<PoinT>);
pcl::PointCloud<PoinT>::Ptr transformed_cloud (new pcl::PointCloud<PoinT>);
pcl::visualization::PCLVisualizer viewer ("PCA registration");

double calculateFitnesscore(pcl::PointCloud<PoinT>::Ptr &source, pcl::PointCloud<PoinT>::Ptr &target, Eigen::Matrix4f &transformationmatrix)
	{
		double fitness_score = 0.0;

		  // Transform the input dataset using the final transformation
		pcl::PointCloud<PoinT>::Ptr input_transformed (new pcl::PointCloud<PoinT>);
		//Eigen::Matrix4f trans = transformationmatrix.inverse();
		  pcl::transformPointCloud (*source, *input_transformed, transformationmatrix);

		  pcl::search::KdTree<PoinT>::Ptr tree (new pcl::search::KdTree<PoinT>);
		  tree->setInputCloud(target);

		  std::vector<int> nn_indices (1);
		  std::vector<float> nn_dists (1);

		  // For each point in the source dataset
		  int nr = 0;
		  for (size_t i = 0; i < input_transformed->points.size (); ++i)
		  {
		    Eigen::Vector4f p1 = Eigen::Vector4f (input_transformed->points[i].x, input_transformed->points[i].y, input_transformed->points[i].z, 0);
		    
		    // Find its nearest neighbor in the target
		    tree->nearestKSearch (input_transformed->points[i], 1, nn_indices, nn_dists);

		    // Deal with occlusions (incomplete targets)
		    if (nn_dists[0] <= 10)
		    {
		      Eigen::Vector4f p2 = Eigen::Vector4f (target->points[nn_indices[0]].x, target->points[nn_indices[0]].y, target->points[nn_indices[0]].z, 0);
		      // Add to the fitness score
		      fitness_score += fabs ((p1-p2).squaredNorm ());
		      //fitness_score += nn_dists[0];
		      nr++;
		    }
		  }

		  if (nr > 0)
		    return (fitness_score / nr);
		  else
		    return 0;


	}

void applyPCAregistration(pcl::PointCloud<PoinT>::Ptr &source, Eigen::Vector4f &centroid, Eigen::Matrix4f &transformationmatrix1)
	{
		//1: compute 3D centroid
		//2: center pointcloud to centroid
		//3: compute covariance matrix
		//4: compute eigenvector and eigenvalues
		//5: eigenvector is the transformation matrix

		std::cout << "PCA registration" << std::endl;

		// compute principal direction
		Eigen::Vector4f _centroid;
		pcl::compute3DCentroid(*source, _centroid);

		std::cout << "_centroid" << std::endl;
		std::cout << _centroid << std::endl;
		for (int i= 0; i < source->points.size(); ++i) {
					source->points[i].x = source->points[i].x - _centroid[0];
					source->points[i].y = source->points[i].y - _centroid[1];
					source->points[i].z = source->points[i].z - _centroid[2];
		}

		Eigen::Matrix3f covariance;
		Eigen::Vector4f centroidnull = Eigen::Vector4f::Zero();
		pcl::computeCovarianceMatrixNormalized(*source, centroidnull, covariance);
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
		Eigen::Vector3f eigenvalues = eigen_solver.eigenvalues();
		Eigen::Matrix3f eigenvectors = eigen_solver.eigenvectors();


		//eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));

		std::cout << "covariance : " << std::endl;
		std::cout << covariance << std::endl;
		std::cout << "eigenvectors : " << std::endl;
		std::cout << eigenvectors << std::endl;
		std::cout << "eigenvalues : " << std::endl;
		std::cout << eigenvalues << std::endl;

		Eigen::Matrix3f transformationmatrix0;
		transformationmatrix0 = eigenvectors;
		centroid = Eigen::Vector4f::Identity();
		centroid = _centroid;

		transformationmatrix1 <<  transformationmatrix0(0,0), transformationmatrix0(0,1), transformationmatrix0(0,2), 0,
				transformationmatrix0(1,0), transformationmatrix0(1,1), transformationmatrix0(1,2), 0,
				transformationmatrix0(2,0), transformationmatrix0(2,1), transformationmatrix0(2,2), 0,
			 			 0,0,0,1;

	}

int main (int argc, char** argv)
{

  //----------------------------------------------------------------------------------
  //Read pcd file
  //---------------------------------------------------------------------------------- 
  if (pcl::io::loadPCDFile<PoinT> ("source_test_nonoise.pcd", *source_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file source_cloud.pcd \n");
    return (-1);
  }
  cout << "Loaded " << source_cloud->width * source_cloud->height << " data points "<< endl;

 if (pcl::io::loadPCDFile<PoinT> ("target_test_nonoise.pcd", *target_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file target_test_nonoise.pcd \n");
    return (-1);
  }
  cout << "Loaded " << target_cloud->width * target_cloud->height << " data points "<< endl;

  //----------------------------------------------------------------------------------
  //Make source cloud Red
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
  //Apply PCA transformation
  //----------------------------------------------------------------------------------
  
  Eigen::Vector4f centroid_source, centroid_target;
  Eigen::Matrix4f transformationm_source, transformationm_target, PCAtransformation;

  applyPCAregistration(source_cloud, centroid_source, transformationm_source);
  applyPCAregistration(target_cloud, centroid_target, transformationm_target);

  PCAtransformation = transformationm_target.transpose() * transformationm_source;

  //transform MISSALIGNED cloud to source_cloud
  pcl::transformPointCloud(*target_cloud, *transformed_cloud, PCAtransformation);

  //calculate fitnesscore
  double fitnesscorePCA=0;
  fitnesscorePCA = calculateFitnesscore(target_cloud,source_cloud, PCAtransformation);
  cout << "FitnesscorePCA is : " << fitnesscorePCA  << " meter "<< endl;

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
  while (!viewer.wasStopped ())
  {
	viewer.addCoordinateSystem(0.1,"id",0);
	viewer.addPointCloud(source_cloud, "source");
	viewer.addPointCloud(transformed_cloud, "transformed");
	viewer.addPointCloud(target_cloud, "target");
        viewer.spin();
  }
  

  //pcl::io::savePCDFileASCII ("source_cloud.pcd", *source_cloud);
  //cerr << "Saved " << source_cloud->points.size () << " data points to source_cloud.pcd." << endl;  

  return (0);
}
