#ifndef GL_DRAW_H_
#define GL_DRAW_H_

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "gl_type.h"

namespace armgl {

inline void set_color(const RGB &color) {
  glColor3f(float(color(0)) / 255.0, float(color(1)) / 255.0,
            float(color(2)) / 255.0);
}

inline void set_color(const Vec3f &color) {
  glColor3f(color(0), color(1), color(2));
}

template <typename T>
inline void draw_point(const Vec3f &point, const T &color, float size = 1.0f) {
  glPointSize(size);
  glBegin(GL_POINTS);
  set_color(color);
  glVertex3f(point(0), point(1), point(2));
  glEnd();
}

template <typename T>
inline void draw_line(const Vec3f &start, const Vec3f &end, const T &color,
                      float width = 1.0f) {
  glLineWidth(width);
  glBegin(GL_LINES);
  set_color(color);
  glVertex3f(start(0), start(1), start(2));
  glVertex3f(end(0), end(1), end(2));
  glEnd();
}

}  // namespace armgl

#endif  // GL_DRAW_H_