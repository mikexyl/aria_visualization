#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <rerun.hpp>

#include "aria_viz/visualizer.h"

using namespace gtsam;

namespace aria::viz {

class VisualizerRerun : public Visualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(VisualizerRerun);
  ARIA_POINTER_TYPEDEFS(VisualizerRerun);

  struct Pose3Renderer {
    virtual void render(VisualizerRerun* viz,
                        const std::string& entity_path,
                        const std::vector<Point3>& points,
                        const std::vector<Eigen::Vector4f>& rgba,
                        std::vector<float> radius,
                        bool is_static) {
      viz->drawPointsImpl(entity_path, points, rgba, radius, is_static);
    }
  };

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

  template <typename... Args>
  void setTime(Args... args) {
    rec_->set_time(args...);
  }

  void setTimeNSec(size_t timestamp) override;

  static std::vector<double> getEllipseFromCov(const Eigen::Matrix3d& cov);

  static std::tuple<double, double, double> getEllipseFromCov(
      const Eigen::Matrix2d& cov);

  static std::vector<Point3> generateEllipse(const Eigen::Vector2d& mean,
                                             const Eigen::Matrix2d& cov);

  void drawLines(const std::string& entity_path,
                 const std::vector<std::pair<Point3, Point3>>& points_pairs,
                 Eigen::Vector4f rgba,
                 float radius,
                 const std::vector<std::string>& labels,
                 const std::vector<std::string>& text) override;

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

  void drawPointsImpl(const std::string& entity_path,
                      const std::vector<Point3>& points,
                      const std::vector<Eigen::Vector4f>& rgba,
                      std::vector<float> radius,
                      bool is_static = false) override;

  /**
   * @brief add spdlog messages to rerun at the given level
   *
   * @param level
   */
  void addSpdlogToRerun(spdlog::level::level_enum level);

  rerun::RecordingStream* rec() { return rec_.get(); }

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
      std::stringstream ss;
      ss << entity_path << "/" << label;
      rec_->log(ss.str(), rerun::Scalar(static_cast<double>(value)));
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

  std::shared_ptr<Pose3Renderer> pose3_renderer_;

  std::optional<AgentId> agent_id_;
};

}  // namespace aria::viz