#include "image.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

Image::Image(int width, int height, int channels)
    : width(width), height(height), channels(channels) {
  data.resize(width * height * channels, 0);
}

float &Image::operator()(int x, int y, int c) {
  if (x < 0 || x >= width || y < 0 || y >= height || c < 0 || c >= channels) {
    std::printf("Index out of range: %d %d %d\n", x, y, c);
    throw std::invalid_argument("Index out of range");
  }
  return data[y * width * channels + x * channels + c];
}

const float &Image::operator()(int x, int y, int c) const {
  if (x < 0 || x >= width || y < 0 || y >= height || c < 0 || c >= channels) {
    std::printf("Index out of range: %d %d %d\n", x, y, c);
    throw std::invalid_argument("Index out of range");
  }
  return data[y * width * channels + x * channels + c];
}

int Image::save_to_file(const std::string &filename) {
  char *d = new char[data.size()];
  auto size = data.size();
  for (int i = 0; i < size; ++i) {
    d[i] = std::clamp(((int)(data[i] * 255.0)), 0, 255);
  }
  stbi_flip_vertically_on_write(true);
  int result = stbi_write_jpg(filename.c_str(), width, height, channels, d, 50);
  delete[] d;
  return result;
}

void Image::clear() { std::fill(data.begin(), data.end(), 0); }
