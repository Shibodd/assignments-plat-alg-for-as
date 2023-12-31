#include <nav_msgs/Odometry.h>
#include <chrono>
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include "Renderer.hpp"
#include <pcl/filters/voxel_grid.h>
#include <particle/helper_cloud.h>
#include <Eigen/Dense>

#include "logging.hpp"
#include "stacked_vectors.hpp"
#include "particle/particle_filter.h"

#include "config.hpp"
#include "config_parse.hpp"

#define bestpID "bestp_id"
#define reflectorID "reflector_id"
#define assID "ass_id"

using namespace lidar_obstacle_detection;

static constexpr Color COLOR_PCL(0.1, 0.1, 0.1); // dark gray
static constexpr Color COLOR_PARTICLE(1, 0, 0); // red
static constexpr Color COLOR_MAP_REFLECTOR(1, 1, 0); // yellow
static constexpr Color COLOR_OBS_REFLECTOR(0, 1, 1); // cyan
static constexpr Color COLOR_BESTP(0, 1, 0); // green
static constexpr Color COLOR_ASSOC(0, 1, 0); // green (lower opacity)

std::ofstream myfile;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particles(new pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix2d landmark_covariance;
Eigen::Matrix2Xd map_landmarks;
Renderer renderer;
ParticleFilter pf;

/**
 * @brief Draw all particles
 */
void update_drawn_particles(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<Particle> &particles)
{
  TRACE_FN_SCOPE;
  for (size_t i = 0; i < particles.size(); ++i)
  {
    cloud->points[i].x = particles[i].state.x();
    cloud->points[i].y = particles[i].state.y();
  }
  renderer.updatePointCloud(cloud, "particles");

  // Draw the best particle
  renderer.removeShape(bestpID);
  renderer.addCircle(pf.best_particle().state.x(), pf.best_particle().state.y(), bestpID, 0.8, COLOR_BESTP);
}

/**
 * @brief Show the reflectors in the frame of reference of the best particle
 */
void updateViewerReflector(const Eigen::Matrix2Xd &observed_landmarks)
{
  TRACE_FN_SCOPE;
  MAKE_FN_LOGGER;

  // Visualize the reflectors the LiDAR sees
  for (Eigen::Index i = 0; i < observed_landmarks.cols(); i++)
  {
    auto& pos = observed_landmarks.col(i);

    // Update the pose of the reflectors
    Eigen::Affine3f transform(Eigen::Translation3f(pos.x(), pos.y(), 0));
    renderer.updatePose(reflectorID + std::to_string(i), transform);
    renderer.updateShape(reflectorID + std::to_string(i), 0.8);
  }

  // Hide the unseen reflectors
  for (int i = observed_landmarks.cols(); i < nReflectors; i++)
    renderer.updateShape(reflectorID + std::to_string(i), 0.0);
}

/**
 * @brief Renders the assocations of the best particle.
*/
void updateViewerBestAssociations(const Eigen::Matrix2Xd &observed_landmarks) {
  TRACE_FN_SCOPE;

  static bool created = false;
  if (created) {
    // Remove old lines
    for (Eigen::Index i = 0; i < map_landmarks.cols(); ++i) {
      renderer.removeShape(assID + std::to_string(i));
    }
  }
  created = true;

  // Add new lines
  int i = 0;
  for (auto ass : pf.best_associations) {
    const auto& obs = observed_landmarks.col(std::get<0>(ass));
    const auto& map = map_landmarks.col(std::get<1>(ass));

    renderer.AddLine(assID + std::to_string(i), pcl::PointXYZ(obs.x(), obs.y(), 0), pcl::PointXYZ(map.x(), map.y(), 0), COLOR_ASSOC, 0.7);
    ++i;
  }
}

/**
 * @brief Process the odometry.
 */
void OdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  TRACE_FN_SCOPE;
  static std::chrono::time_point<std::chrono::high_resolution_clock> t_last;

  double speed = msg->twist.twist.linear.x;
  double yaw_rate = msg->twist.twist.angular.z;

  auto t_now = std::chrono::high_resolution_clock::now();

  double dt = std::chrono::duration<double>(t_now - t_last).count();

  // If we have a "last time"
  if (t_last.time_since_epoch().count() > 0) {
    double noise_mul = 1 + cfg.dynamic_sigma_pos_gain * (1 - pf.best_particle().weight);
    pf.prediction(dt, cfg.sigma_pos * noise_mul, -speed, yaw_rate);
  }

  t_last = t_now;
}

/**
 * @brief Process the LiDAR PCL.
 */
void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  TRACE_FN_SCOPE;

  auto t_start = std::chrono::high_resolution_clock::now();

  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Extract landmarks
  Eigen::Matrix2Xd observed_landmarks_local = stackedvec::hstack(extractReflectors(cloud));

  // Update the particle weights
  pf.updateWeights(landmark_covariance, observed_landmarks_local, map_landmarks);

  // Transform all observations from best particle local space to global space
  auto l2g_transform = pf.best_particle().local2global<double>();
  Eigen::Matrix2Xd observed_landmarks_global = stackedvec::apply_transform(l2g_transform, observed_landmarks_local);

  // Show the reflectors in the frame of reference of the best particle
  updateViewerReflector(observed_landmarks_global);

  // Show the associations of the best particle
  updateViewerBestAssociations(observed_landmarks_global);

  // Show the particles in the map
  update_drawn_particles(cloud_particles, pf.particles);

  // Copy the best particle state before resampling
  auto best_particle_state = pf.best_particle().state;

  // Resample the particles
  pf.resample();

  // Log the execution time
  auto t_end = std::chrono::high_resolution_clock::now();
  double delta_t = (std::chrono::duration<double, std::milli>(t_end - t_start).count()) / 1000;

  myfile << best_particle_state.x() << " " << best_particle_state.y() << " " << delta_t << '\n';

  renderer.SpinViewerOnce();
}

int main(int argc, char **argv)
{
  TRACE_FN_SCOPE;
  MAKE_FN_LOGGER;
  
  logging::setLogLevel(logging::LogLevel::Debug);

  logger.info("Starting.");

  logger.info("Starting ROS node.");
  ros::init(argc, argv, "Particle");
  ros::NodeHandle n;

  logger.info("Parsing config.");
  load_config(n);

  landmark_covariance << 
      cfg.sigma_landmark.x(), 0,
      0, cfg.sigma_landmark.y();
  
  logger.info("Loading reflectors.");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReflectors(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("./data/map_reflector.pcd", *cloudReflectors); // cloud with just the reflectors
  map_landmarks = stackedvec::hstack(createMap(cloudReflectors));

  logger.info("Loading base cloud.");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("./data/map_pepperl.pcd", *cloudMap); // total cloud (used for rendering)

  // Reduce the number of points in the map point cloud (for improving the performance of the rendering)
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloudMap);
  vg.setLeafSize(1.0f, 1.0f, 1.0f); // Set the voxel grid
  vg.filter(*cloud_filtered_map);

  // Delete the old results
  remove("./res.txt");

  // Starts the rendering
  logger.info("Starting renderer.");
  renderer.InitCamera(CameraAngle::XY);
  // Clear viewer
  renderer.ClearViewer();

  // Render the map and reflectors
  renderer.RenderPointCloud(cloud_filtered_map, "originalCloud", COLOR_PCL);
  renderer.RenderPointCloud(cloudReflectors, "reflectorCloud", COLOR_MAP_REFLECTOR);

  // Add the reflectors detected by the particles (you can ignore this)
  for (int i = 0; i < nReflectors; i++)
    renderer.addCircle(0, 0, reflectorID + std::to_string(i), 0.2, COLOR_OBS_REFLECTOR);

  // Init the particle filter
  if (cfg.random_initialization) {
    pf.init_random(cfg.min_initial_position, cfg.max_initial_position, cfg.n_particles);
  } else {
    pf.init(cfg.initial_state, cfg.sigma_init, cfg.n_particles);
  }

  // Render all the particles
  for (int i = 0; i < pf.particles.size(); i++)
  {
    pcl::PointXYZ point;
    point.x = pf.particles[i].state.x();
    point.y = pf.particles[i].state.y();
    point.z = 0;
    cloud_particles->push_back(point);
  }
  renderer.RenderPointCloud(cloud_particles, "particles", COLOR_PARTICLE);

  // render the best initial guess as a circle
  renderer.addCircle(cfg.initial_state.x(), cfg.initial_state.y(), bestpID, 0.4, COLOR_BESTP);
  renderer.SpinViewerOnce();

  logger.info("Creating result file.");
  myfile.open("./res.txt", std::ios_base::app);

  // Subscriber
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/auriga_id0_odom", 1, &OdomCb);             // average rate: 31.438hz
  ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/pepperl_id0_cloud", 1, &PointCloudCb); // average rate: 10.728hz
  // To force 10hz replay use: rosbag play --clock --hz=10 out.bag

  logger.info("Ready!");
  ros::spin();
  myfile.close();
}
