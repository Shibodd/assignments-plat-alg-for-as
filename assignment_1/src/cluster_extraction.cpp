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
#include "../include/cli.hpp"
#include <inttypes.h>
#include "../include/my_clustering.hpp"

// #define USE_PCL_LIBRARY
using namespace lidar_obstacle_detection;

inline Eigen::Vector3f v3f(pcl::PointXYZ pt) {
  return Eigen::Vector3f(pt.x, pt.y, pt.z);
}

// This function sets up the custom kdtree using the point cloud
void setupKdtree(pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, my_kdtree::KdTree& tree)
{
  // insert point cloud points into tree
  for (int i = 0; i < cloud->size(); ++i)
  {
    tree.insert(v3f(cloud->at(i)), i);
  }
}


float get_pcl_distance(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl) {
  float min_distance = std::numeric_limits<float>::max();

  auto end = pcl->end();
  for (auto ii = pcl->begin(); ii != end; ++ii) {
    float distance2 = v3f(*ii).squaredNorm();
    if (distance2 < min_distance) {
      min_distance = distance2;
    }
  }

  return std::sqrt(min_distance);
}

bool is_pcl_in_front(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcl) {
  // Ego vehicle forward vector is always (1, 0, 0)
  return std::any_of(pcl->begin(), pcl->end(), [](pcl::PointXYZ pt) {
    return pt.x > 0;
  });
}


void downsample_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const config::config_ty& cfg) {
  static const logging::Logger logger("downsample_point_cloud");
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
  static const logging::Logger logger("crop_point_cloud");
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
  static const logging::Logger logger("remove_ego_vehicle");
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
  static const logging::Logger logger("ground_removal");
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


void clustering(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointIndices>& clusters, const config::config_ty& cfg) {
  static const logging::Logger logger("clustering");

#ifdef USE_PCL_LIBRARY
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cfg.clustering.tolerance);
  ec.setMinClusterSize(cfg.clustering.min_size);
  ec.setMaxClusterSize(cfg.clustering.max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);

  ec.extract(clusters);
#else
  my_kdtree::KdTree kdtree;
  setupKdtree(cloud, kdtree);

  my_clustering::euclideanClustering(clusters, cloud, kdtree, cfg);
#endif

  if (clusters.size() == 0) {
    logger.warn("Found no clusters!");
  } else {
    logger.info("Found %" PRId64 " clusters.");
  }
}


void ProcessAndRenderPointCloud(const config::config_ty& cfg, Renderer &renderer, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
  static const logging::Logger logger("ProcessAndRenderPointCloud");

  // Point cloud preprocessing
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
  crop_point_cloud(filtered_pcl, cfg);
  remove_ego_vehicle(filtered_pcl, cfg);
  downsample_point_cloud(filtered_pcl, cfg);

  if (cfg.ground_removal.render_ground) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>());
    ground_removal(filtered_pcl, cfg, ground);
    renderer.RenderPointCloud(ground, "ground", lidar_obstacle_detection::Color(1, 0, 0));
  } else {
    ground_removal(filtered_pcl, cfg);
  }

  if (!cfg.clustering.enable) {
    logger.debug("Clustering is disabled.");
    renderer.RenderPointCloud(filtered_pcl, "filtered", lidar_obstacle_detection::Color(1, 1, 1));
    return;
  }

  std::vector<pcl::PointIndices> clusters;
  clustering(filtered_pcl, clusters, cfg);

  int clusterId = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cloud_cluster->push_back((*filtered_pcl)[*pit]);
    }
      
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    float distance = get_pcl_distance(cloud_cluster);

    Color color(1.0f, 1.0f, 1.0f);
    if (distance <= 5.0f && is_pcl_in_front(cloud_cluster)) {
      color = Color(1.0f, 0.0f, 0.0f);
    }

    renderer.RenderPointCloud(cloud_cluster, "cluster" + std::to_string(clusterId), color);

    // Create a cluster bounding box
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);

    Box box{minPt.x, minPt.y, minPt.z,
            maxPt.x, maxPt.y, maxPt.z};
    renderer.RenderBox(box, clusterId);
  

    // Display the distance to the cluster
    Eigen::Vector3f midPt = (v3f(maxPt) + v3f(minPt)) / 2;
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << distance;
    renderer.addText(ss.str(), midPt.x(), midPt.y(), midPt.z(), "txtDist" + std::to_string(clusterId));

    ++clusterId;
  }
}

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