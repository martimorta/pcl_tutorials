#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

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
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-3, 3);
  pass.filter (*cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-3, 3);
  pass.filter (*cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1, 1);
  pass.filter (*cloud);
  cout << "After size: " << cloud->points.size () << "" << endl;

  writer.write (argv[2], *cloud, false);

  return (0);
}
