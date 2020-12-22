# CSC417 final project

You can watch our video [here](https://www.youtube.com/watch?v=ZQy6bHo0ImQ).

## Usage Instructions

- Compile the CMake project.
  ```bash
  $ mkdir build
  $ cd build
  $ cmake .. -DCMAKE_BUILD_TYPE=Release
  $ make
  ```
  
- Run the simulation.
  ```bash
  $ ./final
  ```
  NOTE: The simulation will output images to the current working directory.
  You can turn these images into a video by running
  ```bash
  ffmpeg -framerate 60 -i image%03d.jpg output.mp4
  ```

## Acknowledgement

- [Eigen](https://eigen.tuxfamily.org/)
- [stb_image_write](https://github.com/nothings/stb/blob/master/stb_image_write.h)

