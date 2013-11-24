#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

int
main (int argc, char** argv)
{
  pcl::PCDReader                      reader;
  pcl::PCDWriter                      writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  std:string file_path(argv[1]);

  cout << "Reading PCD" << endl;
  reader.read (file_path, *cloud);
  cout << "Before size: " << cloud->width * cloud->height << " (" << pcl::getFieldsList (*cloud) << ")." << endl;

  //Estimate point normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr  tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::Normal>::Ptr      cloud_normals (new pcl::PointCloud<pcl::Normal> ());
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr      inliers_cylinder (new pcl::PointIndices);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.1);
  seg.setRadiusLimits (0.05, 0.2);
  seg.setInputCloud (cloud);
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Extract indicies from the segmentation and filter the cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  extract.filter (*cloud);

  if (cloud->points.empty ()) 
    cout << "Can't find the cylindrical component." << std::endl;
  else
  {
    cout << "Cylinder size: " << cloud->points.size () << endl;
    writer.write (argv[2], *cloud, false);
  }

  return (0);
}

