namespace utils {
  constexpr double PI = 3.14159265358979323846;
  constexpr double TWO_PI = 2 * PI;

  /**
   * Returns the minimum rotation required for moving from angle x to angle y
  */
  inline double angle_difference_radians(double x, double y) {
    double diff = fmod(y - x + PI, TWO_PI) - PI;
    return diff < -PI ? diff + TWO_PI : diff; 
  }
}