#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "aria_visualization/frame_visualizer_rerun.h"

int main() {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  aria::visualization::FrameVisualizerRerun::Params params("test_rerun");
  aria::visualization::FrameVisualizerRerun frame_visualizer_rerun(params);
  frame_visualizer_rerun.setTimeNSec(0);
  frame_visualizer_rerun.visualizeFactors("", graph, values, {0, 0, 0, 0}, 1.0);
  return 0;
}