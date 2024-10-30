#pragma once

#include <rerun.hpp>

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
    error_ = rec_->connect();
    error_.exit_on_failure();

    rec_->log_static(
        "map",
        rerun::ViewCoordinates::RIGHT_HAND_Z_UP);  // Set an up-axis
  }

  virtual ~VisualizerRerun() {}

  void setTimeNSec(size_t timestamp) override;

  static std::vector<double> getEllipseFromCov(const Eigen::Matrix3d& cov);

  static std::tuple<double, double, double> getEllipseFromCov(
      const Eigen::Matrix2d& cov);

  static std::vector<Point3> generateEllipse(const Eigen::Vector2d& mean,
                                             const Eigen::Matrix2d& cov);

  void connectPointsToPoints(
      const std::string& entity_path,
      const std::vector<std::pair<Point3, Point3>>& points_pairs,
      Eigen::Vector4f rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {}) override {
    connectPointsToPoints(entity_path, points_pairs, rgba, radius, labels, {});
  }

  void connectPointsToPoints(
      const std::string& entity_path,
      const std::vector<std::pair<Point3, Point3>>& points_pairs,
      Eigen::Vector4f rgba,
      float radius,
      const std::vector<std::string>& labels,
      const std::vector<std::string>& text);

  void visualizeUncertainty(const std::string& entity_path,
                            const Point2& mean,
                            const Eigen::Matrix2d& cov,
                            const Eigen::Vector4f& rgba,
                            float line_width) override;

  void visualizeUncertainty(const std::string& entity_path,
                            const Point3& mean,
                            const Eigen::Matrix3d& cov,
                            const Eigen::Vector4f& rgba,
                            float line_width) override;

  void visualizeFactors(const std::string& entity_path,
                        const NonlinearFactorGraph& factors,
                        const Values& values,
                        const Eigen::Vector4f& rgba,
                        float line_width,
                        bool show_labels = false) override;

  float visualizePoints(const std::string& entity_path,
                        const std::vector<Point3>& points,
                        const std::vector<Eigen::Vector4f>& rgba,
                        float radius,
                        bool is_static = false) override;

  void visualizePoints(const std::string& entity_path,
                       const std::vector<Point3>& points,
                       const std::vector<Eigen::Vector4f>& rgba,
                       std::vector<float> radius,
                       bool is_static = false) override;

  float visualizePoints(const std::string& entity_path,
                        const std::vector<Point3>& points,
                        const Eigen::Vector4f& rgba,
                        float radius,
                        bool is_static = false) {
    return Visualizer::visualizePoints(
        entity_path, points, rgba, radius, is_static);
  }

  /**
   * @brief add spdlog messages to rerun at the given level
   *
   * @param level
   */
  void addSpdlogToRerun(spdlog::level::level_enum level);

  auto rec() { return rec_.get(); }

  void plotBenchmarkStats();

  void visualizeUncertainty2D(const std::string& entity_path,
                              const std::vector<Point2>& mean,
                              const std::vector<Eigen::Matrix2d>& cov,
                              const Eigen::Vector4f& rgba,
                              bool is_static);

  void visualizeUncertainty2D(const std::string& entity_path,
                              const std::vector<Point3>& mean,
                              const std::vector<Eigen::Matrix3d>& cov,
                              const Eigen::Vector4f& rgba,
                              bool is_static);

  void visualizeUncertainty2D(const std::string& entity_path,
                              const NonlinearFactorGraph& factors,
                              const Values& values,
                              const Eigen::Vector4f& rgba,
                              bool is_static);

  template <typename T>
  void plotLabeledData(const std::string& entity_path, const T& data) {
    for (const auto& [label, value] : data) {
      rec_->log(entity_path + "/" + label, rerun::Scalar(value));
    }
  }

 protected:
  void connectPositions3D(
      const std::string& entity_path,
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& positions,
      const Eigen::Vector4f& rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {},
      bool clear = false,
      const std::vector<std::string>& text = {});

 private:
  std::unique_ptr<rerun::RecordingStream> rec_;
  rerun::Error error_;

  std::optional<AgentId> agent_id_;
};

}  // namespace aria::viz