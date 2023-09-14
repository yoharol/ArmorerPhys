#ifndef GL_WARP_H_
#define GL_WARP_H_

#include <functional>

#include "gl_type.h"
#include "gl_buffer.h"
#include "gl_shader.h"
#include "gl_camera.h"
#include "gl_geom.h"

namespace glrender {

struct Scene;
typedef std::function<void(Scene)> RenderFunc;

struct Scene {
  Light light;
  Camera camera;
  std::vector<RenderFunc> render_funcs;
  std::vector<bool> depth_test;
};

Scene create_scene(Light light, Camera camera) {
  Scene scene;
  scene.light = light;
  scene.camera = camera;
  scene.render_funcs = std::vector<RenderFunc>();
  scene.depth_test = std::vector<bool>();
  return scene;
}

void add_render_func(Scene& scene, RenderFunc func, bool depth_test = true) {
  scene.render_funcs.push_back(func);
  scene.depth_test.push_back(depth_test);
}

void render_scene(Scene scene) {
  for (int i = 0; i < scene.render_funcs.size(); ++i) {
    if (scene.depth_test[i]) {
      glEnable(GL_DEPTH_TEST);
    } else {
      glDisable(GL_DEPTH_TEST);
    }
    scene.render_funcs[i](scene);
  }
}

}  // namespace glrender

#endif  // GL_WARP_H_