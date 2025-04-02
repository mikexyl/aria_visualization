#pragma once

#include <graphviz/cgraph.h>
#include <graphviz/gvc.h>
#include <gtsam/linear/GaussianBayesTree.h>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <opencv2/imgproc.hpp>
#include <rerun.hpp>

#include "aria_viz/collection_adapters.hpp"
#include "aria_viz/visualizer.h"

using namespace gtsam;

namespace aria::viz {

class VisualizerRerun : public Visualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(VisualizerRerun);
  ARIA_POINTER_TYPEDEFS(VisualizerRerun);

  class Params : public Visualizer::Params {
   public:
    Params(std::optional<std::string> app_id = std::nullopt,
           std::optional<std::string> recording_id = std::nullopt) {
      if (recording_id.has_value()) {
        this->recording_id = recording_id.value();
      } else {
        // generate a random recording id
        this->recording_id = "";
      }

      if (app_id.has_value()) {
        this->app_id = app_id.value();
      } else {
        this->app_id = this->recording_id;
      }
    }

    std::string app_id;
    std::string recording_id;
  };

  VisualizerRerun(Params params) : agent_id_(std::nullopt) {
    // Create a new `RecordingStream` which sends data over TCP to the
    // viewer process.
    spdlog::info("Connecting to rerun server as app_id: {}, recording_id: {}",
                 params.app_id,
                 params.recording_id);
    rec_ = std::make_unique<rerun::RecordingStream>(
        rerun::RecordingStream(params.app_id, params.recording_id));
    rec_->connect_tcp().exit_on_failure();

    rec_->log_static(
        "map",
        rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis
  }

  virtual ~VisualizerRerun() {}

  static inline cv::Mat renderDotToCvMat(const std::string& dot_file_path) {
    // 1. Create Graphviz context
    GVC_t* gvc = gvContext();
    if (!gvc) throw std::runtime_error("Failed to create Graphviz context");

    // 2. Read DOT file
    FILE* fp = fopen(dot_file_path.c_str(), "r");
    if (!fp) throw std::runtime_error("Failed to open DOT file");

    Agraph_t* g = agread(fp, nullptr);
    fclose(fp);
    if (!g) {
      gvFreeContext(gvc);
      throw std::runtime_error("Failed to parse DOT file");
    }

    // 3. Layout and render to memory (PNG format)
    gvLayout(gvc, g, "dot");
    char* data = nullptr;
    unsigned int length = 0;
    gvRenderData(gvc, g, "png", &data, &length);

    // 4. Convert rendered data to OpenCV Mat
    std::vector<uchar> buffer(data, data + length);
    cv::Mat image = cv::imdecode(
        buffer, cv::IMREAD_UNCHANGED);  // Can be grayscale or color

    // 5. Cleanup
    gvFreeRenderData(data);
    gvFreeLayout(gvc, g);
    agclose(g);
    gvFreeContext(gvc);

    return image;
  }

  void setTimeNSec(size_t timestamp) override;

  static std::vector<double> getEllipseFromCov(const Eigen::Matrix3d& cov);

  void drawLinesImpl(const std::string& entity_path,
                     const std::vector<std::pair<Point3, Point3>>& points_pairs,
                     const std::vector<Eigen::Vector4f>& rgba,
                     float radius,
                     const std::vector<std::string>& labels,
                     const std::vector<std::string>& text) override;

  void drawUncertaintyImpl2D(const std::string& entity_path,
                             const Point2& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static) override;

  void drawUncertaintyImpl3D(const std::string& entity_path,
                             const Point3& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static) override;

  void drawPointsImpl(const std::string& entity_path,
                      const std::vector<Point3>& points,
                      const std::vector<Eigen::Vector4f>& rgba,
                      const std::vector<float>& radius,
                      const std::vector<std::string>& labels,
                      bool is_static = false) override;

  /**
   * @brief add spdlog messages to rerun at the given level
   *
   * @param level
   */
  void addSpdlogToRerun(spdlog::level::level_enum level);

  rerun::RecordingStream* rec() { return rec_.get(); }

  void plotBenchmarkStats();

  template <typename T>
  void plotLabeledData(const std::string& entity_path, const T& data) {
    for (const auto& [label, value] : data) {
      std::stringstream ss;
      ss << entity_path << "/" << label;
      rec_->log(ss.str(), rerun::Scalar(static_cast<double>(value)));
    }
  }

  void drawScalar(const std::string& entity_path, double value) override {
    LOG_DATA(entity_path, value);
    rec_->log(entity_path, rerun::Scalar(value));
  }

  template <class BayesTree>
  void drawBayesTree(const std::string& entity_path,
                     const BayesTree& bayes_tree,
                     const Eigen::Vector4f& rgba,
                     float line_width = 0.1f,
                     bool is_static = false) {
    std::vector<std::string> cliques;
    std::vector<rerun::components::GraphEdge> edges;
    std::vector<rerun::components::Color> colors;
    for (auto const& [key, clique] : bayes_tree.nodes()) {
      if (!clique) continue;
      cliques.push_back(fmt::format("{}", *clique));
      for (auto const& child : clique->children) {
        if (!child) continue;
        edges.push_back(
            {fmt::format("{}", *clique), fmt::format("{}", *child)});
      }
      // if no parent, then it is the root then it's red
      if (clique->parent() == nullptr) {
        colors.push_back({255, 0, 0, 255});
      } else {
        colors.push_back({255, 255, 255, 255});
      }
    }

    rec_->log_with_static(
        entity_path,
        is_static,
        rerun::GraphNodes(cliques).with_labels(cliques).with_colors(colors));
    rec_->log_with_static(entity_path,
                          is_static,
                          rerun::GraphEdges(edges).with_graph_type(
                              rerun::components::GraphType::Directed));
  }

  void drawBayesTreeEdges(
      const std::string& entity_path,
      std::vector<std::pair<GaussianBayesTreeClique::shared_ptr,
                            GaussianBayesTreeClique::shared_ptr>> edges,
      std::vector<Eigen::Vector4f> rgba,
      float line_width = 0.1f,
      bool is_static = false);

  void drawDotFile(const std::string& entity_path,
                   const std::string& dot_file_path,
                   bool is_static = false) {
    auto image = renderDotToCvMat(dot_file_path);
    drawImage(entity_path, image, is_static);
  }

  void drawImage(const std::string& entity_path,
                 const cv::Mat& image,
                 bool is_static = false) {
    cv::Mat rgba32;
    if (image.type() == CV_8UC3) {
      cv::cvtColor(image, rgba32, cv::COLOR_BGR2RGBA);
    } else if (image.type() == CV_8UC1) {
      cv::cvtColor(image, rgba32, cv::COLOR_GRAY2RGBA);
    } else if (image.type() == CV_8UC4) {
      rgba32 = image;
    } else {
      throw std::runtime_error("Unsupported image type");
    }

    this->rec_->log_with_static(
        entity_path,
        is_static,
        rerun::Image::from_rgba32(image,
                                  {static_cast<uint32_t>(image.cols),
                                   static_cast<uint32_t>(image.rows)}));
  }

  void drawBayesTreeEdges(
      const std::string& entity_path,
      std::vector<GaussianBayesTreeClique::shared_ptr> edges,
      std::vector<Eigen::Vector4f> rgba,
      float line_width = 0.1f,
      bool is_static = false) {
    // Convert the edges to pairs
    std::vector<std::pair<GaussianBayesTreeClique::shared_ptr,
                          GaussianBayesTreeClique::shared_ptr>>
        edges_pairs;
    for (size_t i = 0; i < edges.size() - 1; i++) {
      edges_pairs.push_back({edges[i], edges[i + 1]});
    }

    drawBayesTreeEdges(entity_path, edges_pairs, rgba, line_width, is_static);
  }

 protected:
  void connectPositions3D(
      const std::string& entity_path,
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& positions,
      const std::vector<Eigen::Vector4f>& rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {},
      bool clear = false,
      const std::vector<std::string>& text = {});

 private:
  std::unique_ptr<rerun::RecordingStream> rec_;

  std::optional<AgentId> agent_id_;
};

}  // namespace aria::viz