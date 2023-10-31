#ifndef PATH_HPP
#define PATH_HPP

#include <Eigen/Dense>
#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkNew.h>
#include <vtkProperty.h>

namespace viewer {

class Path {
  vtkNew<vtkPoints> points;
  vtkNew<vtkPolyLine> polyline;
  vtkNew<vtkCellArray> cells;
  vtkNew<vtkPolyData> polydata;
  vtkNew<vtkPolyDataMapper> mapper;
  vtkNew<vtkActor> actor;

public:
  Path(double r, double g, double b, float w);
  void addPoint(Eigen::Vector3d pt);
  inline vtkActor* getActor() const { return actor.Get(); };
};

}

#endif