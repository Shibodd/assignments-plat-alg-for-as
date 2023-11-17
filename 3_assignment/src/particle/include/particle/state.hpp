#ifndef STATE_HPP
#define STATE_HPP

#include <Eigen/Dense>

class state_ty {
	Eigen::Vector3d state_;

public:
  state_ty(double x, double y, double heading) : state_(x, y, heading) { }
  state_ty(Eigen::Vector2d position, double heading) : state_(position.x(), position.y(), heading) { }
  state_ty(Eigen::Vector3d state) : state_(state) { }

	inline double& x() { return state_(0); }
	inline double& y() { return state_(1); }
	inline double& heading() { return state_(2); }

	inline const double& x() const { return state_(0); }
	inline const double& y() const { return state_(1); }
	inline const double& heading() const { return state_(1); }

	inline Eigen::Vector3d& vec() { return state_; }
	inline const Eigen::Vector3d& vec() const { return state_; }
};


#endif // !STATE_HPP