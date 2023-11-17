#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include "particle/CircleFit.h"

int nReflectors = 8;
/**
 * extractReflectors This function extracts the reflectors from the point cloud
 * @param point cloud from the LiDAR
 * @returns The positions of the reflectors
 */
std::vector<Eigen::Vector2d> extractReflectors(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloudReflector;
  for (auto &point : *cloud)
  {
    if (point.intensity > 0.12f)
      cloudReflector.push_back(point);
  }

  // cluster reflector points, circle fitting and compute the center
  // clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloudReflector.makeShared());
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(0.5);
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(500);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudReflector.makeShared());
  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  std::vector<Eigen::Vector2d> ans;
  ans.reserve(nReflectors);

  // for each cluster perform circle fitting
  for (auto &idx : cluster_indices)
  {
    if (ans.size() > nReflectors)
      break;

    // extract points
    std::vector<float> xComponent, yComponent;
    for (auto &j : idx.indices)
    {
      xComponent.push_back(cloudReflector.at(j).x);
      yComponent.push_back(cloudReflector.at(j).y);
    }

    // Circle fitting
    Circle circle;
    circle = CircleFitByCeres(xComponent, yComponent, 0.04);

    float xAvg = 0.0f;
    for (auto &x : xComponent)
      xAvg += x;
    xAvg /= xComponent.size();
    float yAvg = 0.0f;
    for (auto &y : yComponent)
      yAvg += y;
    yAvg /= yComponent.size();

    if (sqrt(xAvg * xAvg + yAvg * yAvg) > sqrt(circle.x * circle.x + circle.y * circle.y))
      continue; // skip reflector, circle center is nearest that average point

    ans.emplace_back(circle.x, circle.y);
  }
  return ans;
}

/**
 * createMap This function creates a map and extracts the reflectors from the point cloud
 * @param cloud point cloud with the reflectors-landmarks
 * @returns The landmark positions.
 */
std::vector<Eigen::Vector2d> createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  tree->setInputCloud(cloud);
  ec.setClusterTolerance(0);

  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(2);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  std::vector<Eigen::Vector2d> ans;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      ans.emplace_back(cloud->points[*pit].x, cloud->points[*pit].y);

  return ans;
}
