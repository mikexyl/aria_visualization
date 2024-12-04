#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "aria_viz/visualizer_rerun.h"

int main() {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  aria::viz::VisualizerRerun::Params params("test_rerun");
  aria::viz::VisualizerRerun visualizer_rerun(params);
  visualizer_rerun.setTimeNSec(0);
  visualizer_rerun.drawFactors("", graph, values, {0, 0, 0, 0}, 1.0);
  return 0;
}