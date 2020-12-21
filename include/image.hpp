#pragma once

#include <string>
#include <vector>

/**
 * An image. Bottom left corner is 0, 0.
 */
class Image {
public:
  const int width;
  const int height;
  const int channels;

  Image(int width, int height, int channels);

  /**
   * Access pixel.
   * @param x Horizontal index, from 0 to width - 1.
   * @param y Vertical index, from 0 to height - 1.
   * @param c Channel, from 0 to channels - 1.
   */
  float &operator()(int x, int y, int c);
  const float &operator()(int x, int y, int c) const;

  int save_to_file(const std::string &filename);

  void clear();

private:
  std::vector<float> data;
};
