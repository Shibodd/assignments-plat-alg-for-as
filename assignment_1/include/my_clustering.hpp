#ifndef MY_CLUSTERING_HPP
#define MY_CLUSTERING_HPP

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include "../include/config.hpp"
#include "../include/tree_utilities.hpp"


namespace my_clustering 
{

void euclideanClustering(
  std::vector<pcl::PointIndices>& clusters,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  const my_kdtree::KdTree& kdtree,
  const config::config_ty& cfg);

} // namespace my_clustering

#endif // !MY_CLUSTERING_HPP