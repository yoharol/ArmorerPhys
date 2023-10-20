#ifndef GL_CAMERA_H_
#define GL_CAMERA_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <GLFW/glfw3.h>

#include "armgl_type.h"

namespace armgl {

enum CameraType {
  Perspective,
  View2d,
};

struct Camera {
  Vec3f position;
  Vec3f lookat;
  Vec3f up;
  Mat4f projection;
  CameraType type;
  float aspect;
  float near;
  float far;
  float fov;
};

inline void update_camera(Camera& camera);

inline Camera create_camera(Vec3f pos, Vec3f lookat, Vec3f up, float aspect) {
  Camera camera;
  camera.position = pos;
  camera.lookat = lookat;
  camera.up = up;
  camera.projection = Eigen::Matrix4f::Identity();
  camera.type = CameraType::Perspective;
  camera.near = 0.5f;
  camera.far = 100.0f;
  camera.fov = 45.0f;
  camera.aspect = aspect;
  update_camera(camera);
  return camera;
}

inline void set_2d_camera(Camera& camera, float left, float right, float bottom,
                          float top) {
  float l = left;
  float r = right;
  float b = bottom;
  float t = bottom + (right - left) / camera.aspect;
  Mat4f projection;
  projection <<  //
      2.0f / (r - l),
      0.0f, 0.0f, -(r + l) / (r - l),                  // x
      0.0f, 2.0f / (t - b), 0.0f, -(t + b) / (t - b),  // y
      0.0f, 0.0f, 0.0f, 0.0f,                          // z
      0.0f, 0.0f, 0.0f, 1.0f;                          // w
  camera.projection = projection;
  camera.type = CameraType::View2d;
}

inline void update_camera(Camera& camera) {
  float yscale = tan(camera.fov / 2.0f) * camera.near * 2.0;
  float xscale = yscale * camera.aspect;

  float n = -camera.near;
  float f = -camera.far;

  Mat4f to_camera_space;
  Vec3f z = (camera.position - camera.lookat).normalized();
  Vec3f x = camera.up.cross(z).normalized();
  Vec3f y = z.cross(x).normalized();
  to_camera_space << x.x(), x.y(), x.z(), -x.dot(camera.position),  //
      y.x(), y.y(), y.z(), -y.dot(camera.position),                 //
      z.x(), z.y(), z.z(), -z.dot(camera.position),                 //
      0.0f, 0.0f, 0.0f, 1.0f;

  Mat4f perspective;
  perspective << n, 0.0f, 0.0f, 0.0f,  //
      0.0f, n, 0.0f, 0.0f,             //
      0.0f, 0.0f, n + f, -f * n,       //
      0.0f, 0.0f, 1.0f, 0.0f;

  Mat4f orthographic;
  orthographic << 2.0f / xscale, 0.0f, 0.0f, 0.0f,     //
      0.0f, 2.0f / yscale, 0.0f, 0.0f,                 //
      0.0f, 0.0f, -2.0f / (n - f), (n + f) / (n - f),  //
      0.0f, 0.0f, 0.0f, 1.0f;

  camera.projection = orthographic * perspective * to_camera_space;
}

inline void set_camera(Camera& camera, Vec3f position, Vec3f lookat, Vec3f up,
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
  Vec3f lookat(camera.lookat);
  Vec3f pos(camera.position);
  Vec3f up(camera.up);
  float radius = (lookat - pos).norm();
  Vec3f right = (lookat - pos).cross(up).normalized();
  Vec3f forward = (lookat - pos).normalized();
  Vec3f top = right.cross(forward).normalized();
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

  float lookat_speed = 0.5f;
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
    lookat += top * speed * delta_time * lookat_speed;
  }
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
    lookat -= top * speed * delta_time * lookat_speed;
  }
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
    lookat -= right * speed * delta_time * lookat_speed;
  }
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
    lookat += right * speed * delta_time * lookat_speed;
  }
  camera.position = pos;
  camera.lookat = lookat;
  update_camera(camera);
}

inline void set_camera_aspect(Camera& camera, float aspect) {
  camera.aspect = aspect;
  update_camera(camera);
}

const Camera default_camera = create_camera(
    {0.0f, 0.0f, 3.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0}, 1.0f);

}  // namespace armgl

#endif  // GL_CAMERA_H_