#include "viewer/Path.hpp"

namespace viewer {

Path::Path(double r, double g, double b, float w)
{
  cells->InsertNextCell(polyline.Get());
  polydata->SetPoints(points.Get());
  polydata->SetLines(cells.Get());
  mapper->SetInputData(polydata.Get());
  actor->SetMapper(mapper.Get());
  actor->GetProperty()->SetColor(r, g, b);
  actor->GetProperty()->SetLineWidth(w);
}

void Path::addPoint(Eigen::Vector3d pt) {
  points->InsertNextPoint(pt(0), pt(1), pt(2));
  polyline->GetPointIds()->InsertNextId(points->GetNumberOfPoints() - 1);

  cells->Initialize();
  cells->InsertNextCell(polyline.Get());
  
  polydata->Modified();
  cells->Modified();
}

} // namespace viewer