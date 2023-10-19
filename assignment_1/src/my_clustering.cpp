#include "../include/my_clustering.hpp"
#include "../include/logging.hpp"
#include <set>
#include <inttypes.h>

inline Eigen::Vector3f v3f(pcl::PointXYZ pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

static void neighbourhood(
  Eigen::Vector3f point,
  const my_kdtree::Node* node,
  std::vector<int>& neighbour_indices,
  std::set<int>& taken,
  float threshold,
  int depth = 0)
{
  static const logging::Logger logger("neighbourhood");

  /* Perform a Depth-first traversal of the tree to add any point that is close enough. */
  if ((node->point - point).norm() <= threshold)
    neighbour_indices.push_back(node->id);

  int dimension_idx = depth % 3;
  float cut_pos = node->point[dimension_idx];
  float point_pos = point[dimension_idx];

  if (node->left && point_pos - threshold <= cut_pos) {
    neighbourhood(point, node->left.get(), neighbour_indices, taken, threshold, depth + 1);
  }
  if (node->right && point_pos + threshold >= cut_pos) {
    neighbourhood(point, node->right.get(), neighbour_indices, taken, threshold, depth + 1);
  }
}

static void cluster_flood_fill(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const my_kdtree::KdTree& kdtree, int index, std::vector<int>& cluster, std::set<int>& taken, const config::config_ty& cfg) {
  // Too big clusters should result in no clusters
  if (cluster.size() <= cfg.clustering.max_size)
    cluster.push_back(index);
  taken.insert(index);

  std::vector<int> neighbour_indices;
  neighbourhood(v3f(cloud->at(index)), kdtree.root.get(), neighbour_indices, taken, cfg.clustering.tolerance);

  for (int neighbour_index : neighbour_indices) {
    if (taken.find(neighbour_index) == taken.end()) {
      cluster_flood_fill(cloud, kdtree, neighbour_index, cluster, taken, cfg);
    }
  }
}


namespace my_clustering
{

void euclideanClustering(
  std::vector<pcl::PointIndices>& clusters,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  const my_kdtree::KdTree& kdtree,
  const config::config_ty& cfg)
{
  static const logging::Logger logger("euclideanClustering");

  std::set<int> taken;

  for (int i = 0; i < cloud->size(); ++i) {
    // For each point that is not taken yet
    if (taken.find(i) == taken.end()) {
      // Create a new cluster
      clusters.emplace_back();
      auto& cluster = clusters.back();

      // Compute the cluster
      cluster_flood_fill(cloud, kdtree, i, cluster.indices, taken, cfg);  

      // If the cluster is invalid, throw it away
      if (cluster.indices.size() <= cfg.clustering.min_size || cluster.indices.size() > cfg.clustering.max_size) {
        logger.debug("Dropping cluster with %" PRId64 " points", cluster.indices.size());
        clusters.pop_back();
      }
    }
  }
}


} // namespace my_clustering