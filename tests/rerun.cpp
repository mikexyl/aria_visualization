#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/dataset.h>

#include <filesystem>

#include "aria_viz/visualizer_rerun.h"

// customize Marginals class to expose the bayes tree
class MarginalsExposeBayesTree : public gtsam::Marginals {
 public:
  MarginalsExposeBayesTree(const gtsam::NonlinearFactorGraph& graph,
                           const gtsam::Values& values)
      : gtsam::Marginals(graph, values) {}

  const gtsam::GaussianBayesTree& getBayesTree() const { return bayesTree_; }
};

int main(int argc, char** argv) {
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

  std::vector<std::pair<gtsam::GaussianBayesTreeClique::shared_ptr,
                        gtsam::GaussianBayesTreeClique::shared_ptr>>
      edges;
  // collect the path of each clique to its first child
  auto current = bayes_tree.roots().front();
  while (current->children.size() > 0) {
    auto child = current->children.front();
    edges.push_back({current, child});
    current = child;
  }
  spdlog::info("Edges: {}", edges.size());

  visualizer_rerun.drawBayesTree(
      "bayes_tree", bayes_tree, {0, 0, 0, 0.9}, 1.0, false);

  visualizer_rerun.drawBayesTreeEdges(
      "bayes_tree", edges, {{255, 255, 100, 0.9}}, 1.0, false);

  const std::vector<gtsam::Point3> tetra_vertices = {
      {0.0, 0.0, 0.8},
      {-0.6, -0.6, 0.0},
      {0.6, -0.6, 0.0},
      {0.0, 0.6, 0.0},
  };
  const std::vector<aria::viz::MeshTriangle> tetra_triangles = {
      {0, 1, 2},
      {0, 2, 3},
      {0, 3, 1},
      {1, 3, 2},
  };
  const std::vector<Eigen::Vector4f> tetra_colors = {
      {255, 64, 64, 255},
      {64, 255, 64, 255},
      {64, 64, 255, 255},
      {255, 220, 64, 255},
  };
  visualizer_rerun.drawTf("meshes/procedural",
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          0.5,
                          true);
  visualizer_rerun.drawMesh("meshes/procedural",
                            tetra_vertices,
                            tetra_triangles,
                            tetra_colors,
                            {},
                            false);

  const std::vector<gtsam::Point3> textured_quad_vertices = {
      {-1.0, -1.0, 0.0},
      {1.0, -1.0, 0.0},
      {1.0, 1.0, 0.0},
      {-1.0, 1.0, 0.0},
  };
  const std::vector<aria::viz::MeshTriangle> textured_quad_triangles = {
      {0, 1, 2},
      {0, 2, 3},
  };
  const std::vector<aria::viz::MeshTexcoord> textured_quad_uvs = {
      {0.0f, 1.0f},
      {1.0f, 1.0f},
      {1.0f, 0.0f},
      {0.0f, 0.0f},
  };
  cv::Mat textured_quad_image(2, 2, CV_8UC3);
  textured_quad_image.at<cv::Vec3b>(0, 0) = cv::Vec3b(0, 0, 255);
  textured_quad_image.at<cv::Vec3b>(0, 1) = cv::Vec3b(0, 255, 0);
  textured_quad_image.at<cv::Vec3b>(1, 0) = cv::Vec3b(0, 255, 255);
  textured_quad_image.at<cv::Vec3b>(1, 1) = cv::Vec3b(255, 0, 0);

  visualizer_rerun.drawTf(
      "meshes/textured",
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(-3.0, 0.0, 0.0)),
      0.5,
      true);
  visualizer_rerun.drawTexturedMesh("meshes/textured",
                                    textured_quad_vertices,
                                    textured_quad_triangles,
                                    textured_quad_uvs,
                                    textured_quad_image,
                                    {},
                                    {},
                                    false);
  spdlog::info("Logged textured procedural mesh");

  if (argc > 1) {
    const std::filesystem::path mesh_path(argv[1]);
    visualizer_rerun.drawTf(
        "meshes/asset",
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0)),
        0.5,
        true);
    visualizer_rerun.drawMeshFile("meshes/asset", mesh_path, false);
    spdlog::info("Logged mesh asset from {}", mesh_path.string());
  } else {
    spdlog::info("No mesh file argument provided, only logging procedural mesh");
  }

  return 0;
}
