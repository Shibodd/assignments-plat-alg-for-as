#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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


void downsample_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const config::config_ty& cfg) {
  static logging::Logger logger("downsample_point_cloud");
  if (not cfg.voxel_filtering.enable) {
    logger.debug("Downsampling is disabled.");
    return;
  }

  size_t old_size = cloud->size();

  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(cfg.voxel_filtering.leaf_size_x, cfg.voxel_filtering.leaf_size_y, cfg.voxel_filtering.leaf_size_z);
  voxel_grid.filter(*cloud);

  logger.debug("Downsampled cloud from %zu points to %zu.", old_size, cloud->size());
}

void crop_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const config::config_ty& cfg) {
  static logging::Logger logger("crop_point_cloud");
  if (not cfg.crop_cloud.enable) {
    logger.debug("Cloud cropping is disabled.");
    return;
  }

  size_t old_size = cloud->size();

  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(cloud);
  crop_box.setMin(Eigen::Vector4f(cfg.crop_cloud.min_x, cfg.crop_cloud.min_y, cfg.crop_cloud.min_z, 1));
  crop_box.setMax(Eigen::Vector4f(cfg.crop_cloud.max_x, cfg.crop_cloud.max_y, cfg.crop_cloud.max_z, 1));
  crop_box.filter(*cloud);

  logger.debug("Cropped cloud from %zu points to %zu.", old_size, cloud->size());
}

void remove_ego_vehicle(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const config::config_ty& cfg) {
  static logging::Logger logger("remove_ego_vehicle");
  if (not cfg.remove_ego_vehicle.enable) {
    logger.debug("Removing ego vehicle is disabled.");
    return;
  }

  size_t old_size = cloud->size();
  
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(cloud);
  crop_box.setMin(Eigen::Vector4f(cfg.remove_ego_vehicle.min_x, cfg.remove_ego_vehicle.min_y, cfg.remove_ego_vehicle.min_z, 1));
  crop_box.setMax(Eigen::Vector4f(cfg.remove_ego_vehicle.max_x, cfg.remove_ego_vehicle.max_y, cfg.remove_ego_vehicle.max_z, 1));
  crop_box.setNegative(true);
  crop_box.filter(*cloud);

  logger.debug("Removed ego vehicle from cloud - from %zu points to %zu.", old_size, cloud->size());
}

void ground_removal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const config::config_ty &cfg, pcl::PointCloud<pcl::PointXYZ>::Ptr ground = 0) {
  static logging::Logger logger("ground_removal");
  if (not cfg.ground_removal.enable) {
    logger.debug("Ground removal is disabled.");
    return;
  }

  size_t old_size = cloud->size();

  /*
    I observed that using too high values for distanceThreshold results in a plane that is too high (in terms of Z coordinate),
    and therefore some points below the plane are not considered inliers.
    This doesn't make any sense - once we've found a plane that represents the ground,
    anything below it should be considered ground.
    
    Fit a plane with RANSAC, then filter out any point below the plane.
  */
  
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // If true, after finding the plane with the most inliers, it would fit a new plane to the previously found inliers - we don't need it
  seg.setOptimizeCoefficients(false);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(cfg.ground_removal.sac_iterations);
  seg.setDistanceThreshold(cfg.ground_removal.distance_threshold);
  seg.setInputCloud(cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() <= 0) {
    logger.warn("No ground plane detected!");
    return;
  }

  /* To filter stuff below the plane, we will use the PassThrough filter,
     but in the general case we'd have to rotate the pointcloud to make the plane horizontal.
     Just assume the plane is already horizontal, which should work given that the area is decently flat.
     In that case, normal.X and normal.Y are near to zero, and normal.Z is near either -1 or 1
  */
  float normal_z = coefficients->values[2];
  if (fabs(normal_z) <= 0.95f) {
    // Just warn the user, but go on
    logger.warn("Ground plane is not horizontal! Cosine: %f", normal_z);
  }

  float height = coefficients->values[3];
  // Flip the sign if the plane normal points down instead of up
  height *= copysign(1.0f, normal_z);

  pcl::PassThrough<pcl::PointXYZ> pass_through;
  pass_through.setInputCloud(cloud);
  pass_through.setFilterFieldName("z");
  // "pull" the plane up by ground_offset
  pass_through.setFilterLimits(cfg.ground_removal.ground_offset - height, FLT_MAX);
  
  // Apply the filter
  if (ground != 0) {
    pass_through.setNegative(true);
    pass_through.filter(*ground);
  }
  pass_through.setNegative(false);
  pass_through.filter(*cloud);
  
  logger.debug("Removed ground. From %zu points to %zu.", old_size, cloud->size());
}


void ProcessAndRenderPointCloud(const config::config_ty& cfg, Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  static logging::Logger logger("ProcessAndRenderPointCloud");

  // Point cloud preprocessing
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
  crop_point_cloud(filtered_pcl, cfg);
  remove_ego_vehicle(filtered_pcl, cfg);
  downsample_point_cloud(filtered_pcl, cfg);
  ground_removal(filtered_pcl, cfg);

  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>());
  ground_removal(filtered_pcl, cfg, ground);
  renderer.RenderPointCloud(ground, "ground", lidar_obstacle_detection::Color(1, 0, 0));
  */
  
  // Clustering
  std::vector<pcl::PointIndices> cluster_indices;
#ifdef USE_PCL_LIBRARY
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(filtered_pcl);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cfg.clustering.tolerance);
  ec.setMinClusterSize(cfg.clustering.min_size);
  ec.setMaxClusterSize(cfg.clustering.max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered_pcl);

  ec.extract(cluster_indices);
#else
  // Optional assignment
  my_pcl::KdTree treeM;
  treeM.set_dimension(3);
  setupKdtree(filtered_pcl, &treeM, 3);
  cluster_indices = euclideanCluster(filtered_pcl, &treeM, clusterTolerance, setMinClusterSize, setMaxClusterSize);
#endif

  if (cluster_indices.size() == 0) {
    logger.info("Found no clusters!");
  }

  // Render each cluster as a separate point cloud
  const std::vector<Color> colors = { 
    Color(0, 0, 1),
    Color(0, 1, 0),
    Color(0, 1, 1),
    Color(1, 0, 0),
    Color(1, 0, 1),
    Color(1, 1, 0)
  };

  /*
   Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices.

   To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.
   Compute euclidean distance
  */

  renderer.RenderPointCloud(filtered_pcl, "filtered");

  int clusterId = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->push_back((*filtered_pcl)[*pit]);
      
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    renderer.RenderPointCloud(cloud_cluster, "cluster" + std::to_string(clusterId), colors[clusterId % colors.size()]);

    // Here we create the bounding box on the detected clusters
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    // TODO: 8) Here you can plot the distance of each cluster w.r.t ego vehicle
    Box box{minPt.x, minPt.y, minPt.z,
            maxPt.x, maxPt.y, maxPt.z};
    
    // TODO: 9) Here you can color the vehicles that are both in front and 5 meters away from the ego vehicle
    // please take a look at the function RenderBox to see how to color the box
    renderer.RenderBox(box, clusterId);

    ++clusterId;
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
    //renderer.RenderPointCloud(input_cloud, "test_pcl");
    auto endTime = std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    logger.debug("ProcessAndRenderPointCloud took %" PRId64 " milliseconds.", elapsedTime.count());

    streamIterator++;
    if (streamIterator == stream.end()) {
      logging::setLogLevel(logging::LogLevel::Info);
      streamIterator = stream.begin();
    }

    renderer.SpinViewerOnce();
  }


  logger.info("Exiting.");
  return 0;
}