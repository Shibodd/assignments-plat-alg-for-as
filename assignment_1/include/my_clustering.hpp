#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../include/config.hpp"


namespace my_clustering 
{

std::vector<pcl::PointCloud<pcl::PointXYZ>> euclideanClustering(
  const my_kdtree::KdTree kdtree,
  const config::config_ty& cfg);

} // namespace my_clustering