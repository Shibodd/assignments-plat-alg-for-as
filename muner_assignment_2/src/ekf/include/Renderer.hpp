
//
// The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
// His code has been slightly modified to make it more structured.
//

#ifndef LIDAR_OBSTACLE_DETECTION_RENDERER_HPP
#define LIDAR_OBSTACLE_DETECTION_RENDERER_HPP


#include "Box.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <vector>
#include <string>
#include <pcl/io/obj_io.h> 


namespace lidar_obstacle_detection
{

  struct Color
  {

    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {}
  };

  static bool lidarActivated;

  enum class CameraAngle
  {
    XY, TopDown, Side, FPS
  };

  class Renderer
  {
  private:
    unsigned long long rays_counter_;

  public:
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    bool updatePose (const std::string &id, const Eigen::Affine3f& pose);

    static void setLidarStatus();

    static bool getLidarStatus();

    Renderer();

    static void keyboardCallback(const pcl::visualization::KeyboardEvent &event);

    void addCarPos(float x, float y,int id,pcl::PolygonMesh &texture_mesh);

    void removeShape(int id);

    void RenderHighway();

    void RenderRays(const Eigen::Vector3f& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    void addCircle(float centroid_x, float centroid_y,std::string id, float radius, Color& color);
    
    void ClearRays();

    void ClearViewer();

    void addText(float centroid_x, float centroid_y,int id);

    void RenderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          const std::string& name,
                          const Color& color = Color(1,1,1));

    void RenderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                          const std::string& name,
                          const Color& color = Color(-1,-1,-1));

    void RenderBox(const Box& box, int id, const Color& color = Color(1,0,0), float opacity = 1.0);

    void RenderBox(const BoxQ& box, int id, const Color& color = Color(1,0,0), float opacity = 1.0);

    void InitCamera(CameraAngle view_angle);

    bool WasViewerStopped() const;

    void SpinViewerOnce() const;
    void addPointToViewer(float x, float y,std::string source,int coun,Color color);

  };
}

#endif
