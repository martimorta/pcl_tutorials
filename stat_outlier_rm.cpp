#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// remove outliers
#include <pcl/filters/statistical_outlier_removal.h>

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

  // remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (10);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud);
  cout << "After size: " << cloud->points.size () << "" << endl;

  writer.write (argv[2], *cloud, false);

  return (0);
}
