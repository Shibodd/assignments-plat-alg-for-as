#include <set>
#include "../include/my_clustering.hpp"
#include "../include/tree_utilities.hpp"

static void proximity(
  Eigen::Vector3f point,
  const my_kdtree::Node* node,
  std::vector<Eigen::Vector3f>& cluster,
  std::set<const my_kdtree::Node*>& taken,
  float threshold,
  int depth)
{
  float threshold2 = threshold * threshold;
  std::stack<int> stack;
  stack.push(0);

  while (stack.size() > 0) {
    // If the node is close enough and it has not already been taken
    if ((node->point - point).squaredNorm() <= threshold2 
        && taken.find(node) == taken.end()) {

      // Add it to the cluster and mark it as taken
      cluster.push_back(node->point);
      taken.insert(node);
    }

    // Depth-first traversal
    int cut_dimension = depth % my_kdtree::DIMENSIONS;
    int traversal_flag = stack.top();

    // The traversal flag marks whether we need to visit left (0), right (1) or we're done (2).
    // Here it's always either 0 or 1.

  try_side:
    const my_kdtree::Node* child = traversal_flag == 0? node->left.get() : node->right.get();
    if (std::abs(child->point[cut_dimension] - point[cut_dimension]) <= threshold) {
      node = child;
      stack.push(0);
    } else {
      ++traversal_flag;
      goto try_side;
    }

    if (traversal_flag >= 2) {
      // This node's done
      node = node->parent;
      stack.pop();
    }
  }
}


namespace my_clustering
{

std::vector<pcl::PointCloud<pcl::PointXYZ>> euclideanClustering(
  const my_kdtree::KdTree kdtree,
  const config::config_ty& cfg)
{
  
}


} // namespace my_clustering