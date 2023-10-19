#include "../include/my_clustering.hpp"
#include "../include/logging.hpp"
#include <set>
#include <inttypes.h>

inline Eigen::Vector3f v3f(pcl::PointXYZ pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

static void proximity(
  Eigen::Vector3f point,
  const my_kdtree::Node* node,
  std::vector<int>& cluster_indices,
  float threshold)
{
  static const logging::Logger logger("proximity");

  /* Perform a Depth-first traversal of the tree to add any point that is close enough. */

  int depth = 0;
  std::stack<int> stack;
  stack.push(0);

  while (stack.size() > 0) {
    // If the node is close enough, add it to the cluster
    if ((node->point - point).norm() <= threshold)
      cluster_indices.push_back(node->id);

    // Depth-first traversal
    int cut_dimension = depth % my_kdtree::DIMENSIONS;
    int& traversal_flag = stack.top();
    

    // The traversal flag marks whether we need to visit left (0), right (1) or we're done (2).
    // Here it's always either 0 or 1.
    bool descending = false;
    for (; traversal_flag < 2; ++traversal_flag) {
      const my_kdtree::Node* child = traversal_flag == 0? node->left.get() : node->right.get();

      if (child == nullptr)
        continue;

      float cut_pos = child->point[cut_dimension];
      float pt_pos = point[cut_dimension];

      bool left = traversal_flag == 0 && pt_pos - threshold <= cut_pos;
      bool right = traversal_flag == 1 && pt_pos + threshold >= cut_pos;

      if (left || right) {
        // Then we need to search on this side of the hyperplane
        ++traversal_flag;

        // From the next iteration...
        node = child;  // ... we'll start traversing this child
        stack.push(0); // ... starting at the left node
        descending = true;
        ++depth;
        break;
      }
    }

    if (!descending && traversal_flag >= 2) {
      // We traversed both left and right - this node's done, so ascend
      node = node->parent; // unwind the recursion
      stack.pop(); // remove our traversal flag
      --depth;
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
      proximity(v3f(cloud->at(i)), kdtree.root.get(), cluster.indices, cfg.clustering.tolerance);

      // Add the cluster to the taken set
      for (int index : cluster.indices)
        taken.insert(index);

      // If the cluster is invalid, throw it away
      if (cluster.indices.size() <= cfg.clustering.min_size || cluster.indices.size() > cfg.clustering.max_size) {
        logger.debug("Dropping cluster with %" PRId64 " points", cluster.indices.size());
        clusters.pop_back();
      }
    }
  }
}


} // namespace my_clustering