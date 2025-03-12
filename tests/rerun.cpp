#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include "aria_viz/visualizer_rerun.h"

// customize Marginals class to expose the bayes tree
class MarginalsExposeBayesTree : public gtsam::Marginals {
 public:
  MarginalsExposeBayesTree(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::Values& values)
      : gtsam::Marginals(graph, values) {}

  const gtsam::GaussianBayesTree& getBayesTree() const { return bayesTree_; }
};

int main() {
  auto [graph, values] =
      gtsam::readG2o("smallGrid3D.g2o", true, gtsam::KernelFunctionTypeHUBER);
  // add prior on first pose
  gtsam::Pose3 priorMean = gtsam::Pose3();
  gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1).finished());
  graph->add(gtsam::PriorFactor<gtsam::Pose3>(0, priorMean, priorNoise));

  // optimize with Gauss-Newton
  gtsam::LevenbergMarquardtOptimizer optimizer(*graph, *values);
  *values = optimizer.optimize();

  aria::viz::VisualizerRerun::Params params("test_rerun");
  aria::viz::VisualizerRerun visualizer_rerun(params);
  visualizer_rerun.setTime();
  visualizer_rerun.drawFactors("graph", *graph, *values, {0, 0, 0, 0.9}, 1.0);
  MarginalsExposeBayesTree marginals(*graph, *values);
  const auto& bayes_tree = marginals.getBayesTree();
  visualizer_rerun.drawBayesTree(
      "bayes_tree", bayes_tree, {0, 0, 0, 0.9}, 1.0, false);
  return 0;
}