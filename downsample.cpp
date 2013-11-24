#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
// downsample using voxelgrid
#include <pcl/filters/voxel_grid.h>

using namespace std;

int
main (int argc, char** argv)
{
  pcl::PCDReader                      reader;
  pcl::PCDWriter                      writer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  std:string file_path(argv[1]);
  float leaf_size = atof(argv[3]);

  cout << "Reading PCD" << endl;
  reader.read (file_path, *cloud);
  cout << "Before size: " << cloud->width * cloud->height << " (" << pcl::getFieldsList (*cloud) << ")." << endl;

  // downsample using voxelgrid
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.05f, 0.05f, 0.05f);
  vg.filter (*cloud);
  cout << "After size: " << cloud->points.size () << "" << endl;

  writer.write (argv[2], *cloud, false);

  return (0);
}
