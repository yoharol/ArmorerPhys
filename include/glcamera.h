#ifndef GL_CAMERA_H_
#define GL_CAMERA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <glfw/glfw3.h>

namespace glrender {

enum CameraType {
  Perspective,
  Orthographic,
};

struct Camera {
  Eigen::Vector3f position;
  Eigen::Vector3f lookat;
  Eigen::Vector3f up;
  Eigen::Matrix4f view;
  Eigen::Matrix4f projection;
  CameraType type;
  float aspect;
  float near;
  float far;
  float fov;
};

inline Camera create_camera() {
  Camera camera;
  camera.position = Eigen::Vector3f(0.0f, 0.0f, 3.0f);
  camera.lookat = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  camera.up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
  camera.projection = Eigen::Matrix4f::Identity();
  camera.type = CameraType::Perspective;
  camera.near = 0.1f;
  camera.far = 500.0f;
  camera.fov = 45.0f;
  camera.aspect = 1.0f;
  return camera;
}

inline void update_camera(Camera& camera) {
  float yscale = tan(camera.fov / 2.0f) * camera.near * 2.0;
  float xscale = yscale * camera.aspect;

  float n = -camera.near;
  float f = -camera.far;

  Eigen::Matrix4f to_camera_space;
  Eigen::Vector3f z = (camera.position - camera.lookat).normalized();
  Eigen::Vector3f x = camera.up.cross(z).normalized();
  Eigen::Vector3f y = z.cross(x).normalized();
  to_camera_space << x.x(), x.y(), x.z(), -x.dot(camera.position),  //
      y.x(), y.y(), y.z(), -y.dot(camera.position),                 //
      z.x(), z.y(), z.z(), -z.dot(camera.position),                 //
      0.0f, 0.0f, 0.0f, 1.0f;

  Eigen::Matrix4f perspective;
  perspective << n, 0.0f, 0.0f, 0.0f,  //
      0.0f, n, 0.0f, 0.0f,             //
      0.0f, 0.0f, n + f, -f * n,       //
      0.0f, 0.0f, 1.0f, 0.0f;

  Eigen::Matrix4f orthographic;
  orthographic << 2.0f / xscale, 0.0f, 0.0f, 0.0f,     //
      0.0f, 2.0f / yscale, 0.0f, 0.0f,                 //
      0.0f, 0.0f, 2.0f / (n - f), -(n + f) / (n - f),  //
      0.0f, 0.0f, 0.0f, 1.0f;

  camera.projection = orthographic * perspective * to_camera_space;
}

inline void set_camera(Camera& camera, Eigen::Vector3f position,
                       Eigen::Vector3f lookat, Eigen::Vector3f up,
                       CameraType type = CameraType::Perspective) {
  camera.position = position;
  camera.lookat = lookat;
  camera.up = up;
  camera.type = type;
  update_camera(camera);
}

inline void orbit_camera_control(GLFWwindow* window, Camera& camera,
                                 float speed = 1.0f,
                                 float delta_time = 1.0f / 60.0f) {
  Eigen::Vector3f lookat(camera.lookat);
  Eigen::Vector3f pos(camera.position);
  Eigen::Vector3f up(camera.up);
  float radius = (lookat - pos).norm();
  Eigen::Vector3f right = (lookat - pos).cross(up).normalized();
  Eigen::Vector3f forward = (lookat - pos).normalized();
  Eigen::Vector3f top = right.cross(forward).normalized();
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    pos += forward * speed * delta_time;
    camera.position = pos;
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    pos -= forward * speed * delta_time;
  }
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    pos -= right * speed * delta_time;
    pos = lookat + (pos - lookat).normalized() * radius;
  }
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    pos += right * speed * delta_time;
    pos = lookat + (pos - lookat).normalized() * radius;
  }
  if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
    pos += top * speed * delta_time;
    pos = lookat + (pos - lookat).normalized() * radius;
  }
  if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
    pos -= top * speed * delta_time;
    pos = lookat + (pos - lookat).normalized() * radius;
  }
  camera.position = pos;
  update_camera(camera);
}

inline void set_camera_aspect(Camera& camera, float aspect) {
  camera.aspect = aspect;
  update_camera(camera);
}

}  // namespace glrender

#endif  // GL_CAMERA_H_