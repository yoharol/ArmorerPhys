#ifndef GL_CAMERA_H_
#define GL_CAMERA_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "ArmorerPhys/type.h"

namespace aphys {

enum CameraType {
  Perspective,
  Orthographic,
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

void update_camera(Camera& camera);

Camera create_camera(Vec3f pos, Vec3f lookat, Vec3f up, float aspect);

void set_2d_camera(Camera& camera, float left, float right, float bottom,
                   float top);

void camera2d_screen_to_world(Camera& camera, float& x, float& y);

void update_camera(Camera& camera);

void set_camera(Camera& camera, Vec3f position, Vec3f lookat, Vec3f up,
                CameraType type = CameraType::Perspective);

void orbit_camera_control(GLFWwindow* window, Camera& camera,
                          float speed = 1.0f, float delta_time = 1.0f / 60.0f);

inline void set_camera_aspect(Camera& camera, float aspect) {
  camera.aspect = aspect;
  update_camera(camera);
}

extern const Camera default_camera;

}  // namespace aphys

#endif  // GL_CAMERA_H_