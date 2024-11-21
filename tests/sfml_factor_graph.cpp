#include <aria_viz/visualizer_sfml.h>

using namespace aria::viz;

int main() {
  aria::viz::VisualizerSFML::Params params;
  aria::viz::VisualizerSFML viz(params);

  while (not viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  NonlinearFactorGraph graph;
  Values values;

  for (int i = 0; i < 10; i++) {
    values.insert(i, Pose3(Rot3(), Point3(rand() % 800, rand() % 600, 0)));
  }

  for (int i = 0; i < 9; i++) {
    Pose3 pose1 = values.at<Pose3>(i), pose2 = values.at<Pose3>(i + 1),
          between = pose1.between(pose2);
    graph.emplace_shared<BetweenFactor<Pose3>>(
        i, i + 1, between, noiseModel::Unit::Create(6));
  }
  std::vector<Eigen::Vector4f> rgba(values.size(),
                                    Eigen::Vector4f(255, 0, 0, 255));
  viz.drawPoints(
      "samples", values, {Eigen::Vector4f(255, 0, 0, 255)}, {10}, true);
  viz.drawFactors(
      "graph", graph, values, Eigen::Vector4f(255, 0, 0, 255), 2, true);

  viz.render();
  // wait for window close
  while (viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}