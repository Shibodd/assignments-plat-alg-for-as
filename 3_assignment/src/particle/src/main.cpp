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

#include "particle/particle_filter.h"

/*
 * TODO
 * Define the proper number of particles
 */
#define NPARTICLES 0
#define circleID "circle_id"
#define reflectorID "reflector_id"

using namespace lidar_obstacle_detection;

std::vector<Eigen::Vector2d> map_reflectors;

ParticleFilter pf;
Renderer renderer;
std::vector<Particle> best_particles;
std::ofstream myfile;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_particles(new pcl::PointCloud<pcl::PointXYZ>);

/*
 * TODO
 * Define the proper noise values
 */
Eigen::Vector3d sigma_init(0, 0, 0);         //[x,y,theta] initialization noise.
Eigen::Vector3d sigma_pos(0.05, 0.05, 0.05); //[x,y,theta] movement noise. Try values between [0.5 and 0.01]
Eigen::Vector2d sigma_landmark(0.4, 0.4);    //[x,y] sensor measurement noise. Try values between [0.5 and 0.1]
std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1), Color(1, 0, 1), Color(0, 1, 1)};

// This function updates the position of the particles in the viewer
void showPCstatus(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<Particle> &particles)
{
  for (size_t i = 0; i < particles.size(); ++i)
  {
    cloud->points[i].x = particles[i].state.x();
    cloud->points[i].y = particles[i].state.y();
  }
  renderer.updatePointCloud(cloud, "particles");
}


/**
 * @brief Show the reflectors in the frame of reference of the best particle
 */ 
void updateViewerReflector(const std::vector<Eigen::Vector2d>& observed_landmarks)
{
  auto bestp_l2g_transform = best_particles.back().local2global<float>();

  // Visualize the reflectors the LiDAR sees
  for (int i = 0; i < observed_landmarks.size(); i++)
  {
    // Compute the position of the reflector in global space
    Eigen::Vector2f pos = bestp_l2g_transform * observed_landmarks[i];

    // Update the pose of the reflectors
    Eigen::Affine3f transform(Eigen::Translation3f(pos.x(), pos.y(), 0));
    renderer.updatePose(reflectorID + std::to_string(i), transform);
    renderer.updateShape(reflectorID + std::to_string(i), 1.0);
  }

  // Hide the unseen reflectors
  for (int i = observed_landmarks.size(); i < nReflectors; i++)
    renderer.updateShape(reflectorID + std::to_string(i), 0.0);
}

// This functions processes the odometry from the forklift (Prediction phase)
void OdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
  static std::chrono::time_point<std::chrono::high_resolution_clock> t_last;

  double speed = msg->twist.twist.linear.x;
  double yaw_rate = msg->twist.twist.angular.z;

  auto t_now = std::chrono::high_resolution_clock::now();

  double dt = (std::chrono::duration<double, std::milli>(t_now - t_last).count()) / 1000;

  // If we have a "last time"
  if (t_last.time_since_epoch().count() > 0)
    pf.prediction(dt, sigma_pos, speed, yaw_rate);

  t_last = t_now;
}

// This functions processes the point cloud (Update phase)
void PointCloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  auto t_start = std::chrono::high_resolution_clock::now();

  // Convert to PCL data type
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Extract landmarks
  std::vector<Eigen::Vector2d> observed_landmarks = extractReflectors(cloud);

  // Show the reflectors in the frame of reference of the best particle
  updateViewerReflector(observed_landmarks);

  // Update the weights of the particle
  pf.updateWeights(sigma_landmark, observed_landmarks, map_mille);

  // Resample the particles
  pf.resample();

  // Calculate and output the average weighted error of the particle filter over all time steps so far.
  Particle best_particle;
  vector<Particle> particles = pf.particles;
  double highest_weight = -1.0;
  for (int i = 0; i < particles.size(); ++i)
  {
    if (particles[i].weight > highest_weight)
    {
      highest_weight = particles[i].weight;
      best_particle = particles[i];
    }
  }
  best_particles.push_back(best_particle);

  // Show the particles in the map
  showPCstatus(cloud_particles, particles);
  renderer.removeShape(circleID + std::to_string(NPARTICLES + 1));
  renderer.addCircle(best_particles.back().x(), best_particles.back().y(), circleID + std::to_string(NPARTICLES + 1), 0.3, 1, 0, 0);

  // Log the execution time
  auto t_end = std::chrono::high_resolution_clock::now();
  double delta_t = (std::chrono::duration<double, std::milli>(t_end - t_start).count()) / 1000;

  myfile << best_particle.x << " " << best_particle.y << " " << delta_t << '\n';

  renderer.SpinViewerOnce();
}

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudReflectors(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("./data/map_reflector.pcd", *cloudReflectors); // cloud with just the reflectors
  map_reflectors = createMap(cloudReflectors);

  // Load base cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile("./data/map_pepperl.pcd", *cloudMap);          // total cloud (used for rendering)

  // Reduce the number of points in the map point cloud (for improving the performance of the rendering)
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_map(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(cloudMap);
  vg.setLeafSize(1.0f, 1.0f, 1.0f); // Set the voxel grid
  vg.filter(*cloud_filtered_map);

  // Delete the old results
  remove("./res.txt");

  // Starts the rendering
  renderer.InitCamera(CameraAngle::XY);
  // Clear viewer
  renderer.ClearViewer();

  // Render the map and reflectors
  renderer.RenderPointCloud(cloud_filtered_map, "originalCloud", colors[2]);
  renderer.RenderPointCloud(cloudReflectors, "reflectorCloud", colors[0]);

  // Add the reflectors detected by the particles (you can ignore this)
  for (int i = 0; i < nReflectors; i++)
    renderer.addCircle(0, 0, reflectorID + std::to_string(i), 0.2, 1, 1, 1);

  // Initial position of the forklift
  double GPS_x = 2.37256;
  double GPS_y = 1.70077;
  double GPS_theta = -1.68385;

  // Insert one particle in the best particle set
  Particle p(GPS_x, GPS_y, GPS_theta);
  best_particles.push_back(p);

  // Init the particle filter
  pf.init(GPS_x, GPS_y, GPS_theta, sigma_init, NPARTICLES);
  // pf.init_random(sigma_init,NPARTICLES);

  // Render all the particles
  for (int i = 0; i < NPARTICLES; i++)
  {
    pcl::PointXYZ point;
    point.x = pf.particles[i].x();
    point.y = pf.particles[i].y();
    point.z = 0;
    cloud_particles->push_back(point);
  }
  renderer.RenderPointCloud(cloud_particles, "particles", colors[0]);

  // render the best initial guess as a circle
  renderer.addCircle(GPS_x, GPS_y, circleID + std::to_string(NPARTICLES + 1), 0.4, 0, 1, 1);
  renderer.SpinViewerOnce();

  // Start ROS node
  std::cout << "Map loaded, waiting for the rosbag" << std::endl;
  myfile.open("./res.txt", std::ios_base::app);

  ros::init(argc, argv, "Particle");
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/auriga_id0_odom", 1, &OdomCb);             // average rate: 31.438hz
  ros::Subscriber pc_sub = n.subscribe<sensor_msgs::PointCloud2>("/pepperl_id0_cloud", 1, &PointCloudCb); // average rate: 10.728hz
  // To force 10hz replay use: rosbag play --clock --hz=10 out.bag
  ros::spin();
  myfile.close();
}
