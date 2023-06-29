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
};

Scene create_scene(Light light, Camera camera) {
  Scene scene;
  scene.light = light;
  scene.camera = camera;
  scene.render_funcs = std::vector<RenderFunc>();
  return scene;
}

void add_render_func(Scene &scene, RenderFunc func) {
  scene.render_funcs.push_back(func);
}

void render_scene(Scene scene) {
  for (RenderFunc func : scene.render_funcs) {
    func(scene);
  }
}

}  // namespace glrender

#endif  // GL_WARP_H_