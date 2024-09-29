#pragma once

#include <aria_common/logging.h>
#include <aria_common/macros.h>
#include <aria_common/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <optional>

using namespace gtsam;

namespace aria::viz {

struct ColorMap {
  static Eigen::Vector3f random() {
    // generate a random vector from 0 to 256
    float r = rand() % 256;
    float g = rand() % 256;
    float b = rand() % 256;
    return Eigen::Vector3f(r, g, b);
  }

  static Eigen::Vector4f random(float alpha) {
    return Eigen::Vector4f(random()(0), random()(1), random()(2), alpha);
  }

  static const Eigen::Vector3f kGreen;
  static const Eigen::Vector3f kRed;
  static const Eigen::Vector3f kBlue;
  static const Eigen::Vector3f kGray;
  static const Eigen::Vector3f kBlack;
};

struct AgentColorMap : public ColorMap {
  static std::map<int, Eigen::Vector3f> color_map;

  static Eigen::Vector4f get(AgentIdPair agent_id_pair, float alpha) {
    auto [agent0, agent1] = agent_id_pair;
    if (color_map.find(agent0) == color_map.end()) {
      color_map[agent0] = random();
    }
    if (color_map.find(agent1) == color_map.end()) {
      color_map[agent1] = random();
    }

    // blend the two colors
    auto color0 = color_map[agent0];
    auto color1 = color_map[agent1];
    auto color = (color0 + color1) / 2;
    return Eigen::Vector4f(color(0), color(1), color(2), alpha);
  }
};

class Visualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(Visualizer);
  ARIA_POINTER_TYPEDEFS(Visualizer);

  class Params {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Params() {}

    bool visualize_keypoints{true};
    bool visualize_keypoints_tracking{true};
    bool visualize_landmarks_2d_tracking{false};
    bool tracking_side_by_side{false};
    bool step_by_step{false};
    bool visualize_ba_edges{false};

    bool any_visualization_enabled() const {
      return visualize_keypoints || visualize_landmarks_2d_tracking ||
             visualize_keypoints_tracking;
    }

    void turnOffAll() {
      visualize_keypoints = false;
      visualize_keypoints_tracking = false;
      visualize_landmarks_2d_tracking = false;
      tracking_side_by_side = false;
      step_by_step = false;
      visualize_ba_edges = false;
    }
  };

  // generate color as float numbers from 0 to 1
  static Vector3 generateRandomColorf() {
    float r = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float g = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    float b = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
    return Vector3(r, g, b);
  }

  static cv::Scalar toCvColor(const Eigen::Vector3f& color) {
    return cv::Scalar(color(0) * 255, color(1) * 255, color(2) * 255);
  }

  static cv::Scalar toCvColor(const Eigen::Vector3d& color) {
    return cv::Scalar(color(0) * 255, color(1) * 255, color(2) * 255);
  }

  Visualizer(Params param = {}) : params_(param) {
    // //TODO: why this not working?
    // // create a empty image to create the window
    // cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
    // cv::namedWindow("landmarks2d", cv::WINDOW_AUTOSIZE);
    // cv::imshow("landmarks2d", image);
    // cv::waitKey(1);
  }

  virtual ~Visualizer() {}

  virtual void setTimeNSec(size_t timestamp) {}

  void toggleStepByStep() { params_.step_by_step = !params_.step_by_step; }

  virtual void connectPointsToPoints(
      const std::string& entity_path,
      const std::vector<std::pair<Point3, Point3>>& points_pairs,
      Eigen::Vector4f rgba,
      float radius = 0.01f,
      const std::vector<std::string>& labels = {}) {}

  float visualizePoints(const std::string& entity_path,
                        const std::vector<Point3>& points,
                        const Eigen::Vector4f& rgba,
                        float radius,
                        bool is_static = false) {
    std::vector<Eigen::Vector4f> rgbs(points.size(), rgba);
    return visualizePoints(entity_path, points, rgbs, radius, is_static);
  }

  virtual float visualizePoints(const std::string& entity_path,
                                const std::vector<Point3>& points,
                                const std::vector<Eigen::Vector4f>& rgba,
                                float radius,
                                bool is_static = false) {
    return 0.;
  }

  virtual void visualizePoints(const std::string& entity_path,
                               const std::vector<Point3>& points,
                               const std::vector<Eigen::Vector4f>& rgba,
                               std::vector<float> radius,
                               bool is_static = false) {}

  static std::optional<Point3> getPoint3(const Key& key, const Values& values) {
    auto dim = values.at(key).dim();
    if (values.exists(key) == false) {
      return std::nullopt;
    }
    if (dim == 3) {
      auto point = values.at<Pose2>(key);
      return Point3(point.x(), point.y(), 0);
    } else if (dim == 6) {
      auto point = values.at<Pose3>(key);
      return Pose3(point).translation();
    } else {
      LOG_FATAL("Not implemented for dim: " + std::to_string(dim));
    }
  }

  /**
   * @brief highlight keys with a line pointing upwards
   *
   * @param entity_path
   * @param keys
   * @param values
   * @param rgba
   * @param radius
   * @param is_static
   */
  void highlightKeys(const std::string& entity_path,
                     const KeySet& keys,
                     const Values& values,
                     const Eigen::Vector4f& rgba,
                     float radius,
                     bool is_static = false) {
    std::vector<std::pair<Point3, Point3>> points;
    for (auto key : keys) {
      auto point = getPoint3(key, values);
      if (point) {
        Point3 p_up{point->x(), point->y(), point->z() + radius * 10.};
        points.emplace_back(*point, p_up);
      }
    }

    connectPointsToPoints(entity_path, points, rgba, radius);
  }

  template <typename ContainerT>
  void visualizeGTCameraPoses(const std::string& entity_path,
                              const ContainerT& frames);

  virtual void visualizeUncertainty(const std::string& entity_path,
                                    const Point2& mean,
                                    const Eigen::Matrix2d& cov,
                                    const Eigen::Vector4f& rgba,
                                    float line_width) {}

  virtual void visualizeUncertainty(const std::string& entity_path,
                                    const Point3& mean,
                                    const Eigen::Matrix3d& cov,
                                    const Eigen::Vector4f& rgba,
                                    float line_width) {}

  virtual void visualizeFactors(const std::string& entity_path,
                                const NonlinearFactorGraph& factors,
                                const Values& values,
                                const Eigen::Vector4f& rgba,
                                float line_width,
                                bool show_labels = false) {}

 protected:
  virtual void step() {
    // set a keyboard callback for R
    int key = cv::waitKey(params_.step_by_step ? 0 : 1) & 0xFF;
    if (key == 'r') {
      toggleStepByStep();
    }
  }

 protected:
  Params params_;
};
}  // namespace aria::viz
