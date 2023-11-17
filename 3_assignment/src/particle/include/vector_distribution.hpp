#ifndef VECTOR_DIST_HPP
#define VECTOR_DIST_HPP

// Welcome to macro hell

template <template <typename> typename Distribution, typename DataType, int Size>
class VectorDistribution
{
  std::array<Distribution<DataType>, Size> dists;

public:
  // holy macroroni
  template <typename... ArgType>
  VectorDistribution(Eigen::Matrix<ArgType, Size, 1>... args)
  {
    for (int i = 0; i < Size; ++i)
      dists[i] = Distribution<DataType>((args(i))...); // this is actually pretty cool
  }

  template <typename Generator>
  Eigen::Matrix<DataType, Size, 1> operator()(Generator generator)
  {
    Eigen::Matrix<DataType, Size, 1> ans;
    for (int i = 0; i < Size; ++i)
      ans[i] = dists[i](generator);
    return ans;
  }
};

// Makes the compiler use its brain instead of having you specify the size template argument
template <template <typename> typename Distribution, typename DataType, int Size, typename... ArgType>
inline VectorDistribution<Distribution, DataType, Size> make_vector_distribution(Eigen::Matrix<ArgType, Size, 1>... args)
{
  return VectorDistribution<Distribution, DataType, Size>(args...);
}

#endif // !VECTOR_DISTRIBUTION
