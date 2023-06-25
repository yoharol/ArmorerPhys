#ifndef GL_TYPE_H_
#define GL_TYPE_H_

#include <Eigen/Core>

namespace glrender {

using MatXf =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vec3f = Eigen::Vector3f;
using RGB = Eigen::Vector3i;

}  // namespace glrender

#endif  // GL_TYPE_H_