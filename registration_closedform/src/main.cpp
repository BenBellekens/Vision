#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
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
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

bool PCAregistration = false;
bool SVDregistration = true;
double FitnesscorePCA=0;
double FitnesscoreSVD=0;

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

void applySVDregistration(pcl::PointCloud<PoinT>::Ptr &source, pcl::PointCloud<PoinT>::Ptr &target, Eigen::Matrix4f &transformationmatrix)
	{
		//center both pointcloud
		Eigen::Vector4f _centroid_source, _centroid_target;
		pcl::compute3DCentroid(*source, _centroid_source);
		pcl::compute3DCentroid(*target, _centroid_target);

		for (int i= 0; i < source->points.size(); ++i) {
			source->points[i].x = source->points[i].x - _centroid_source[0];
			source->points[i].y = source->points[i].y - _centroid_source[1];
			source->points[i].z = source->points[i].z - _centroid_source[2];
		}
		for (int i= 0; i < target->points.size(); ++i) {
			target->points[i].x = target->points[i].x - _centroid_target[0];
			target->points[i].y = target->points[i].y - _centroid_target[1];
			target->points[i].z = target->points[i].z - _centroid_target[2];
		}
		//check if the point count is equal
		std::cout << "source points " << source->points.size() << " target points " << target->points.size() << std::endl;
		pcl::PointCloud<PoinT>::Ptr temp (new pcl::PointCloud<PoinT>);
		*temp = *source;

		for (int i = 0; i < source->points.size(); ++i) {
			if(i < target->points.size()){
				temp->points[i].x = target->points[i].x;
				temp->points[i].y = target->points[i].y;
				temp->points[i].z = target->points[i].z;
			}
			else{
				temp->points[i].x = source->points[i].x;
				temp->points[i].y = source->points[i].y;
				temp->points[i].z = source->points[i].z;
			}
		}
		std::cout << "source points " << source->points.size() << " target points " << temp->points.size() << std::endl;
		std::cout << "start svd estimation" << std::endl;
		pcl::registration::TransformationEstimationSVD<PoinT, PoinT> trans_est;
		Eigen::Matrix4f svdtransformation;
		trans_est.estimateRigidTransformation(*source, *temp, transformationmatrix);

		svdtransformation = transformationmatrix;

	}

void applyPCAregistration(pcl::PointCloud<PoinT>::Ptr &source, Eigen::Vector4f &centroid, Eigen::Matrix4f &transformationmatrix1)
	{
		//1: compute 3D centroid
		//2: center pointcloud to centroid
		//3: compute covariance matrix
		//4: compute eigenvector and eigenvalues
		//5: eigenvector is the transformation matrix

		std::cout << "PCA registration" << std::endl;
		//pcl::PointCloud<PoinT>::Ptr temp_cloud (new pcl::PointCloud<PoinT>);
		//*temp_cloud = *source;
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

//----------------------------------------------------------------------------------
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
  //Apply PCA transformation from target to source
  //----------------------------------------------------------------------------------
  if(PCAregistration == true){
	Eigen::Vector4f centroid_source, centroid_target;
	Eigen::Matrix4f transformationm_source, transformationm_target, PCAtransformation;

	applyPCAregistration(source_cloud, centroid_source, transformationm_source);
	applyPCAregistration(target_cloud, centroid_target, transformationm_target);
	PCAtransformation = transformationm_source * transformationm_target.transpose();

	//Apply rotation transformation 
	pcl::transformPointCloud(*source_cloud, *transformed_cloud, PCAtransformation);
	
	//calculate fitnesscore
	FitnesscorePCA = calculateFitnesscore(target_cloud,source_cloud, PCAtransformation);
	cout << "FitnesscorePCA is : " << FitnesscorePCA  << " meter "<< endl;
  }

  //----------------------------------------------------------------------------------
  //Apply SVD transformation from target to source
  //----------------------------------------------------------------------------------
  if(SVDregistration == true){	
	Eigen::Matrix4f trans_matrix_svd;
	applySVDregistration(source_cloud, target_cloud, trans_matrix_svd);
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, trans_matrix_svd);

	//calculate fitnesscore
 	FitnesscoreSVD = calculateFitnesscore(target_cloud,source_cloud, trans_matrix_svd);
	cout << "FitnesscoreSVD is : " << FitnesscoreSVD  << " meter "<< endl;
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
	//while keypress "q" is pressed
  }
  

  return (0);
}
