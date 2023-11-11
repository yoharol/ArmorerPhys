#ifndef GL_TYPE_H_
#define GL_TYPE_H_

#include <Eigen/Core>

namespace aphys {

using MatxXf =
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using MatxXi =
    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vecxf = Eigen::VectorXf;
using Vecxi = Eigen::VectorXi;
using Vec4f = Eigen::Vector4f;
using Vec3f = Eigen::Vector3f;
using Vec2f = Eigen::Vector2f;
using Mat2f = Eigen::Matrix2f;
using Mat3f = Eigen::Matrix3f;
using Mat4f = Eigen::Matrix4f;
using RGB = Eigen::Vector3i;
using Listx3f = std::vector<Vec3f>;
using Listx2f = std::vector<Vec2f>;
using Matx3f = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;
using Matx2f = Eigen::Matrix<float, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Matx3i = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>;

}  // namespace aphys

#endif  // GL_TYPE_H_