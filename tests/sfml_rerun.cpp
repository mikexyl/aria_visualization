#include <aria_viz/visualizer_rerun.h>
#include <gtsam/geometry/Pose3.h>

#include <rerun.hpp>

using namespace aria::viz;

// WIP

int main() {
  aria::viz::VisualizerRerun::Params params;
  aria::viz::VisualizerRerun viz(params);

  int it = 0;
  while (true) {
    viz.setTime("sfml", std::chrono::system_clock::now());
    std::vector<Point3> samples;
    for (int i = 0; i < 10; i++) {
      samples.push_back(Point3(i * 10, i * 10, 0));
    }

    viz.drawPoints(
        "samples", samples, {Eigen::Vector4f(255, 0, 0, 255)}, 10, true);

    if (it++ > 10) break;
  }

  return 0;
}
