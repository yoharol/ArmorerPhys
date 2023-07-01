#ifndef GL_TYPE_H_
#define GL_TYPE_H_

#include <Eigen/Core>

namespace glrender {

using MatXf =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vec4f = Eigen::Vector4f;
using Vec3f = Eigen::Vector3f;
using Vec2f = Eigen::Vector2f;
using Mat4f = Eigen::Matrix4f;
using RGB = Eigen::Vector3i;
using List3f = std::vector<Vec3f>;
using List2f = std::vector<Vec2f>;
using Mat3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Mat2f = Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Mat3i = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

}  // namespace glrender

#endif  // GL_TYPE_H_