#include <aria_viz/visualizer_rerun.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>
#include <rerun.hpp>

using namespace aria::viz;

int main() {
  aria::viz::VisualizerRerun::Params params;
  params.pose3_renderer =
      std::make_unique<VisualizerRerun::Pose3RendererSFML>();
  aria::viz::VisualizerRerun viz(std::move(params));

  int it = 0;
  while (true) {
    viz.setTime("sfml", std::chrono::system_clock::now());
    std::vector<Point3> samples;
    for (int i = 0; i < 10; i++) {
      samples.push_back(Point3(i * 10, i * 10, 0));
    }

    viz.visualizePoints(
        "samples", samples, {Eigen::Vector4f(255, 0, 0, 255)}, 10, true);

    if (it++ > 10) break;
  }

  return 0;
}
