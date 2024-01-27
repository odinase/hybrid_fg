#ifndef HYBRID_FG_TYPES_HPP
#define HYBRID_FG_TYPES_HPP

#include <Eigen/Core>

namespace hfg::types
{

template <int Rows, int Cols>
using Matrixd = Eigen::Matrix<double, Rows, Cols>;

template <int Rows, int Cols>
using RowMajorMatrixd = Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor>;

using RowMajorMatrixXd = RowMajorMatrixd<Eigen::Dynamic, Eigen::Dynamic>;

template <int Dim>
using Vectord = Eigen::Vector<double, Dim>;

}  // namespace hfg::types

#endif  // HYBRID_FG_TYPES_HPP