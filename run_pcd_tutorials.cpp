#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// downsample using voxelgrid
#include <pcl/filters/voxel_grid.h>
// remove outliers
#include <pcl/filters/statistical_outlier_removal.h>
// planar segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  cout << "Reading PCD" << endl;
  // Replace the path below with the path where you saved your file
  // reader.read ("/home/mmorta/Experiments/bags/pcl_tests/hd.pcd", *cloud);

  // cout << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // // Perform stuff ---------------------------------------------------------------------------------

  // // remove outliers
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_filtered);

  // // downsample using voxelgrid
  // pcl::VoxelGrid<pcl::PointXYZ> vg;
  // vg.setInputCloud (cloud_filtered);
  // vg.setLeafSize (0.05f, 0.05f, 0.05f);
  // vg.filter (*cloud_filtered);

  // planar segmentation
  // 
  // 
  reader.read ("/home/mmorta/Experiments/bags/pcl_tests/hd_sor_mean50_stddev1_downsampled.pcd", *cloud_filtered);
  cout << "PointCloud before filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  cout << "Planar segmentation" << endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud_filtered);
  seg.segment (*inliers, *coefficients);


  // pcl::ExtractIndices<PointType> eifilter (true); // Initializing with true will allow us to extract the removed indices
  // eifilter.setInputCloud (cloud_in);
  // eifilter.setIndices (*inliers);
  // eifilter.filter (*cloud_filtered);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZ> cloud_filtered_seg;
//  pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_rgb);
  cloud_filtered_seg.width  = inliers->indices.size();
  cloud_filtered_seg.height = 1;

  cloud_filtered_seg.points.reserve(inliers->indices.size());

  for (size_t i = 0; i < inliers->indices.size(); ++i)
  {
    cloud_filtered_seg.points.push_back(cloud_filtered->points[inliers->indices[i]]);
  }

  //  Write results -------------------------------------------------------------------------

  pcl::PCDWriter writer;
  writer.write ("/home/mmorta/Experiments/bags/pcl_tests/hd_sor_vg_seg.pcd", cloud_filtered_seg, false);

  return (0);
}