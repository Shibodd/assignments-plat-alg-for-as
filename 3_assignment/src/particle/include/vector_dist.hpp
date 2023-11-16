#ifndef VECTOR_DIST_HPP
#define VECTOR_DIST_HPP

template<typename DataType, template <typename> typename Distribution, int Size>
class VectorDist {
    std::array<Distribution<DataType>, Size> dists;

public:

    // holy macroroni
    template<typename ...EigenType>
    VectorDist(EigenType... args) {
        for (int i = 0; i < Size; ++i) {
            dists[i] = Distribution((args(i))...);
        }
    }

    template<typename Generator>
    Eigen::Matrix<DataType, Size, 1> operator()(Generator generator) {
        Eigen::Matrix<DataType, Size, 1> ans;
        for (int i = 0; i < Size; ++i)
            ans[i] = dists[i](generator);
        return ans;
    }
};

#endif  // !VECTOR_DIST
