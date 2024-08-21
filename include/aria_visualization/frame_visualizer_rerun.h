#pragma once

#include <aria_net/ext_frame.h>
#include <aria_net/frame_visualizer.h>

#include <rerun.hpp>

using namespace gtsam;
using namespace aria::net;

namespace aria::visualization {

class FrameVisualizerRerun : public FrameVisualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(FrameVisualizerRerun);
  ARIA_POINTER_TYPEDEFS(FrameVisualizerRerun);

  class Params : public FrameVisualizer::Params {
   public:
    Params(std::string app_id,
           std::optional<std::string> recording_id = std::nullopt)
        : app_id(app_id) {
      if (recording_id.has_value()) {
        this->recording_id = recording_id.value();
      } else {
        // generate a random recording id
        this->recording_id = std::to_string(std::rand());
      }
    }
    std::string app_id;
    std::string recording_id;
  };

  FrameVisualizerRerun(Params params) : agent_id_(std::nullopt) {
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

  virtual ~FrameVisualizerRerun() {}

  static std::vector<Point3> generateEllipse(const Eigen::Vector2d& mean,
                                             const Eigen::Matrix2d& cov);

  void visualize(const Frame::Ptr& frame) override;

  void setTimeNSec(size_t timestamp) override;

  void visualizeExtFrame(const ExtFrame::Ptr& ext_frame,
                         size_t timestamp) override;

  void connectFrames(
      const std::string& entity_path,
      Eigen::Vector4f rgba,
      std::vector<std::pair<Frame::Ptr, Frame::Ptr>>& frame_pairs) override;

  void connectLandmarks(const std::string& entity_path,
                        Eigen::Vector4f rgba,
                        std::vector<LandmarkPair>& landmark_pairs) override;

  void connectFramesToLandmarks(
      const std::string& entity_path,
      Eigen::Vector4f rgba,
      std::map<Landmark::Ptr, FrameSet>& landmarks_and_frames) override;

  void connectPointsToPoints(
      const std::string& entity_path,
      const std::vector<std::pair<Point3, Point3>>& points_pairs,
      Eigen::Vector4f rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {}) override;

  void visualizeUncertainty(const std::string& entity_path,
                            const Point2& mean,
                            const Eigen::Matrix2d& cov,
                            const Eigen::Vector4f& rgba,
                            float line_width) override;

  void visualizeFactors(const std::string& entity_path,
                        const NonlinearFactorGraph& factors,
                        const Values& values,
                        const Eigen::Vector4f& rgba,
                        float line_width) override;

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

 protected:
  void visualizeTrackingImage(const cv::Mat& image,
                              const Frame::Ptr& frame) override;

  void visualizeLandmarks(const std::map<LandmarkId, Point3>& landmarks,
                          std::string frame_id,
                          size_t timestamp) override;

  void visualizeLandmarks(const Frame::Ptr& frame,
                          const FrameDatabase::Ptr& frame_db,
                          bool use_all_frames) override;

  void step() override { FrameVisualizer::step(); }

  void visualizeCameraPose(const Frame::Ptr& frame) override;

  void connectPositions3D(
      const std::string& entity_path,
      const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& positions,
      const Eigen::Vector4f& rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {},
      bool clear = false);

 private:
  std::unique_ptr<rerun::RecordingStream> rec_;
  rerun::Error error_;

  std::optional<AgentId> agent_id_;
};

}  // namespace aria::visualization