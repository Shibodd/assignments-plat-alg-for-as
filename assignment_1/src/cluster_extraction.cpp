#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include "../include/Renderer.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <chrono>
#include <unordered_set>
#include "../include/tree_utilities.hpp"
#include "../include/logging.hpp"
#include "../include/config.hpp"

#define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

typedef std::unordered_set<int> my_visited_set_t;

// This function sets up the custom kdtree using the point cloud
void setupKdtree(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree *tree, int dimension)
{
  // insert point cloud points into tree
  for (int i = 0; i < cloud->size(); ++i)
  {
    tree->insert({cloud->at(i).x, cloud->at(i).y, cloud->at(i).z}, i);
  }
}

/*
OPTIONAL
This function computes the nearest neighbors and builds the clusters
    - Input:
        + cloud: Point cloud to be explored
        + target_ndx: i-th point to visit
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + visited: Visited points --> typedef std::unordered_set<int> my_visited_set_t;
        + cluster: Here we add points that will represent the cluster
        + max: Max cluster size
    - Output:
        + visited: already visited points
        + cluster: at the end of this function we will have one cluster
*/
void proximity(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int target_ndx, my_pcl::KdTree *tree, float distanceTol, my_visited_set_t &visited, std::vector<int> &cluster, int max)
{
  if (cluster.size() < max)
  {
    cluster.push_back(target_ndx);
    visited.insert(target_ndx);

    std::vector<float> point{cloud->at(target_ndx).x, cloud->at(target_ndx).y, cloud->at(target_ndx).z};

    // get all neighboring indices of point
    std::vector<int> neighborNdxs = tree->search(point, distanceTol);

    for (int neighborNdx : neighborNdxs)
    {
      // if point was not visited
      if (visited.find(neighborNdx) == visited.end())
      {
        proximity(cloud, neighborNdx, tree, distanceTol, visited, cluster, max);
      }

      if (cluster.size() >= max)
      {
        return;
      }
    }
  }
}

/*
OPTIONAL
This function builds the clusters following a euclidean clustering approach
    - Input:
        + cloud: Point cloud to be explored
        + tree: kd tree for searching neighbors
        + distanceTol: Distance tolerance to build the clusters
        + setMinClusterSize: Minimum cluster size
        + setMaxClusterSize: Max cluster size
    - Output:
        + cluster: at the end of this function we will have a set of clusters
TODO: Complete the function
*/
std::vector<pcl::PointIndices> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, my_pcl::KdTree *tree, float distanceTol, int setMinClusterSize, int setMaxClusterSize)
{
  my_visited_set_t visited{};              // already visited points
  std::vector<pcl::PointIndices> clusters; // vector of PointIndices that will contain all the clusters
  std::vector<int> cluster;                // vector of int that is used to store the points that the function proximity will give me back
  // for every point of the cloud
  //   if the point has not been visited (use the function called "find")
  //     find clusters using the proximity function
  //
  //     if we have more clusters than the minimum
  //       Create the cluster and insert it in the vector of clusters. You can extract the indices from the cluster returned by the proximity funciton (use pcl::PointIndices)
  //     end if
  //   end if
  // end for
  return clusters;
}


void downsample_point_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst, float lx, float ly, float lz) {
  static logging::Logger logger("downsample_point_cloud");
  size_t old_size = src->size();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(src);
  voxel_grid.setLeafSize(lx, ly, lz);
  voxel_grid.filter(*dst);

  logger.debug("Downsampled cloud from %zu points to %zu.", old_size, dst->size());
}

void crop_point_cloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst, Eigen::Vector4f min, Eigen::Vector4f max) {
  static logging::Logger logger("crop_point_cloud");
  size_t old_size = src->size();

  pcl::CropBox<pcl::PointXYZ> crop_box(true);
  crop_box.setInputCloud(src);
  crop_box.setMin(min);
  crop_box.setMax(max);
  crop_box.filter(*dst);

  logger.debug("Cropped cloud from %zu points to %zu.", old_size, dst->size());
}

void plane_removal(pcl::PointCloud<pcl::PointXYZ>::ConstPtr src, pcl::PointCloud<pcl::PointXYZ>::Ptr dst, int ransac_iter, float distance_thresh) {
  static logging::Logger logger("plane_removal");
  size_t old_size = src->size();

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(ransac_iter);
  seg.setDistanceThreshold(distance_thresh);
  seg.setInputCloud(src);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ()); 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(src); 
  extract.setIndices(inliers);
  extract.setNegative(true);

  extract.filter(*dst);

  if (inliers->indices.size() <= 0) {
    logger.debug("No plane detected.");
  } else {
    logger.debug("Removed a plane. From %zu points to %zu.", old_size, dst->size());
  }
}


void ProcessAndRenderPointCloud(const config::config_ty& cfg, Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  static logging::Logger logger("ProcessAndRenderPointCloud");

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>);

  // 1) Downsample the dataset
  downsample_point_cloud(cloud, filtered_pcl, cfg.voxel_filtering.leaf_size_x, cfg.voxel_filtering.leaf_size_y, cfg.voxel_filtering.leaf_size_z);

  // 2) here we crop the points that are far away from us, in which we are not interested
  crop_point_cloud(filtered_pcl, filtered_pcl,
    Eigen::Vector4f(cfg.crop_cloud.min_x, cfg.crop_cloud.min_y, cfg.crop_cloud.min_z, 1),
    Eigen::Vector4f(cfg.crop_cloud.max_x, cfg.crop_cloud.max_y, cfg.crop_cloud.max_z, 1)
  );

  // 3) Segmentation and apply RANSAC
  // 4) iterate over the filtered cloud, segment and remove the planar inliers
  plane_removal(filtered_pcl, filtered_pcl, cfg.plane_removal.sac_iterations, cfg.plane_removal.distance_threshold);

  renderer.RenderPointCloud(filtered_pcl, "downsampled");
  return;

  // TODO: 5) Create the KDTree and the vector of PointIndices

  // TODO: 6) Set the spatial tolerance for new cluster candidates (pay attention to the tolerance!!!)
  std::vector<pcl::PointIndices> cluster_indices;

#ifdef USE_PCL_LIBRARY

  // PCL functions
  // HERE 6)
#else
  // Optional assignment
  my_pcl::KdTree treeM;
  treeM.set_dimension(3);
  setupKdtree(filtered_pcl, &treeM, 3);
  cluster_indices = euclideanCluster(filtered_pcl, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
#endif

  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 0, 1), Color(0, 1, 1)};

  /**Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices.

  To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
  Compute euclidean distance
  **/
  int j = 0;
  int clusterId = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->push_back((*filtered_pcl)[*pit]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    renderer.RenderPointCloud(cloud, "originalCloud" + std::to_string(clusterId), colors[2]);
    // TODO: 7) render the cluster and plane without rendering the original cloud
    //<-- here
    //----------

    // Here we create the bounding box on the detected clusters
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    // TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
    Box box{minPt.x, minPt.y, minPt.z,
            maxPt.x, maxPt.y, maxPt.z};
    // TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
    // please take a look at the function RenderBox to see how to color the box
    renderer.RenderBox(box, j);

    ++clusterId;
    j++;
  }
}


#include "../include/cli.hpp"
#include <inttypes.h>

int main(int argc, char *argv[])
{
  // CLI and configuration parsing.
  const auto args = cli::parse_args(argc, argv);
  logging::setLogLevel(args.logLevel);

  const auto cfg = config::parse_config_file(args.config_file_path);

  logging::Logger logger("main");
  logger.info("Starting.");

  Renderer renderer;
  renderer.InitCamera(CameraAngle::XY);
  // Clear viewer
  renderer.ClearViewer();

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{args.point_cloud_frames_directory},
                                              boost::filesystem::directory_iterator{});

  // sort files in ascending (chronological) order
  std::sort(stream.begin(), stream.end());

  auto streamIterator = stream.begin();

  while (not renderer.WasViewerStopped())
  {
    renderer.ClearViewer();

    pcl::PCDReader reader;
    reader.read(streamIterator->string(), *input_cloud);
    logger.debug("Loaded %zu data points from %s.", input_cloud->points.size(), streamIterator->string().c_str());

    auto startTime = std::chrono::steady_clock::now();
    ProcessAndRenderPointCloud(cfg, renderer, input_cloud);
    // renderer.RenderPointCloud(input_cloud, "test_pcl");
    auto endTime = std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    logger.debug("ProcessAndRenderPointCloud took %" PRId64 " milliseconds.", elapsedTime.count());

    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    renderer.SpinViewerOnce();
  }


  logger.info("Exiting.");
  return 0;
}