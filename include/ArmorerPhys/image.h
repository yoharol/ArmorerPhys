#ifndef GL_IMAGE_H_
#define GL_IMAGE_H_

#include <iostream>

namespace aphys {

struct Image {
  unsigned char* data;
  int width;
  int height;
  int nrChannels;
};

Image load_image(const std::string path);

void free_image(Image image);

}  // namespace aphys

#endif  // GL_IMAGE_H_