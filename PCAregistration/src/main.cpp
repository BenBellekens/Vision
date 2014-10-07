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

typedef pcl::PointXYZ PoinT;
using namespace std;

pcl::PointCloud<PoinT>::Ptr source_cloud (new pcl::PointCloud<PoinT>);
pcl::PointCloud<PoinT>::Ptr target_cloud (new pcl::PointCloud<PoinT>);
pcl::visualization::PCLVisualizer viewer ("PCA registration");

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
  if (pcl::io::loadPCDFile<PoinT> ("source_cloud.pcd", *source_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file source_cloud.pcd \n");
    return (-1);
  }
  cout << "Loaded " << source_cloud->width * source_cloud->height << " data points "<< endl;

  //----------------------------------------------------------------------------------
  //initial ROTATION to MISSALIGN the cloud
  //----------------------------------------------------------------------------------
  Eigen::Matrix4f transformationMatrixZ,transformationMatrixY, Transformation;
  double anglerad = pcl::deg2rad(30.0);
  transformationMatrixY << cos(anglerad),0, sin(anglerad) ,0,
 		 0,1, 0,0,
 		 -sin(anglerad),0,cos(anglerad),0,
		 0,0,0,1;

  transformationMatrixZ << cos(0), -sin(anglerad), 0,0,
		 sin(anglerad), cos(anglerad), 0,0,
		 0,0,1,0,
		 0,0,0,1;

  Transformation = transformationMatrixY * transformationMatrixZ;
  pcl::transformPointCloud(*source_cloud, *target_cloud, Transformation);

  //----------------------------------------------------------------------------------
  //initial TRANSLATION to MISSALIGN the cloud
  //----------------------------------------------------------------------------------
  Eigen::Matrix4f affinetranslation = Eigen::Matrix4f::Identity();
  affinetranslation(0,3) = 0.5;
  pcl::transformPointCloud(*target_cloud, *target_cloud, affinetranslation);

  

  //----------------------------------------------------------------------------------
  //Apply PCA transformation
  //----------------------------------------------------------------------------------
  

  while (!viewer.wasStopped ())
  {
	viewer.addCoordinateSystem(0.1,"id",0);
	viewer.addPointCloud(source_cloud, "source");
	viewer.addPointCloud(target_cloud, "target");
        viewer.spin();
  }
  

  //pcl::io::savePCDFileASCII ("target_cloud.pcd", target_cloud);
  //cerr << "Saved " << target_cloud->points.size () << " data points to target_cloud.pcd." << endl;  

  return (0);
}
