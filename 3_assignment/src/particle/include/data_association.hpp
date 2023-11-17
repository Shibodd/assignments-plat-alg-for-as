#ifndef DATA_ASSOCIATION_HPP
#define DATA_ASSOCIATION_HPP

#include <Eigen/Dense>
#include <vector>

std::vector<int> point_association(
    const std::vector<Eigen::Vector2d> &observations,
    const std::vector<Eigen::Vector2d> &map,
    Eigen::Matrix2d covariance);

#endif // !DATA_ASSOCIATION_HPP