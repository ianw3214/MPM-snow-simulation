#include "image.hpp"

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

void Image::save_to_file(const std::string &filename) {
  // TODO
}
