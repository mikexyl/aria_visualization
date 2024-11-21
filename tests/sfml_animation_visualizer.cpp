#include <aria_viz/visualizer_sfml.h>

using namespace aria::viz;

int main() {
  aria::viz::VisualizerSFML::Params params;
  params.wait_for_first_image = false;
  aria::viz::VisualizerSFML viz(params);

  while (not viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Window opened" << std::endl;

  int it = 0;
  while (true) {
    while (viz.frameFinished()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::vector<Point3> samples;
    std::vector<float> radius;
    for (int i = 0; i < 10; i++) {
      samples.push_back(Point3(i * 10, i * 10, 0));
      radius.push_back(10);
    }

    std::cout << "Visualized points" << std::endl;
    viz.visualizePoints(
        "samples", samples, {Eigen::Vector4f(255, 0, 0, 255)}, radius, true);

    if (it++ > 1000) break;
  }

  // wait for window close
  std::cout << "Waiting for window close" << std::endl;
  while (viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}