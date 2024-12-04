#include <aria_viz/visualizer_sfml.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/Marginals.h>

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
    Vector6 noise(20, 20, 20, 0.1, 0.1, 0.1);
    Pose3 noisy_between =
        between.retract(Vector6::Random().array() * noise.array());
    graph.emplace_shared<BetweenFactor<Pose3>>(
        i, i + 1, between, noiseModel::Diagonal::Sigmas(noise));
  }

  // add prior in the first pose
  graph.emplace_shared<PriorFactor<Pose3>>(
      0,
      values.at<Pose3>(0).compose(Pose3(Rot3(), Point3(100, 0, 0))),
      noiseModel::Unit::Create(6));

  LevenbergMarquardtParams params_lm;
  params_lm.setVerbosity("SILENT");
  LevenbergMarquardtOptimizer optimizer(graph, values, params_lm);
  Values result = optimizer.optimize();

  // compute marginals
  Marginals marginals(graph, result);
  for (int i = 0; i < 10; i++) {
    auto marg = marginals.marginalCovariance(i);
    Matrix33 marg_xyz = marg.block<3, 3>(0, 0);
    viz.drawUncertainty(
        "uncertainty", result.at<Pose3>(i), marg_xyz, {125, 125, 0, 100}, 1.0);
  }

  std::vector<Eigen::Vector4f> rgba(values.size(),
                                    Eigen::Vector4f(255, 0, 0, 255));
  viz.drawPoints(
      "samples", values, {Eigen::Vector4f(255, 0, 0, 255)}, {10}, true);
  viz.drawPoints("gt", result, {Eigen::Vector4f(0, 255, 0, 255)}, {10}, true);
  viz.drawFactors(
      "graph", graph, result, Eigen::Vector4f(0, 255, 0, 255), 2, true);

  viz.render();
  // wait for window close
  while (viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}