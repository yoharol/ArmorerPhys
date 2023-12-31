#include "ArmorerPhys/image.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace aphys {

Image load_image(const std::string path) {
  Image image;
  stbi_set_flip_vertically_on_load(1);
  image.data = stbi_load(path.c_str(), &image.width, &image.height,
                         &image.nrChannels, 0);
  if (!image.data) {
    std::cerr << "Failed to load texture" << std::endl;
    exit(-1);
  }
  return image;
}

void free_image(Image image) { stbi_image_free(image.data); }

}  // namespace aphys