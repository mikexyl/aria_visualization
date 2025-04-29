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

  static Eigen::Vector3f random(std::string seed) {
    // set random seed
    std::seed_seq seed_seq(seed.begin(), seed.end());
    std::mt19937 gen(seed_seq);
    std::uniform_int_distribution<> dis(0, 150);

    // generate a random vector from 0 to 256
    float r = dis(gen) + 100;
    float g = dis(gen) + 100;
    float b = dis(gen) + 100;
    return Eigen::Vector3f(r, g, b);
  }

  static Eigen::Vector4f random(float alpha) {
    Eigen::Vector3f color = random();
    return Eigen::Vector4f(color(0), color(1), color(2), alpha);
  }

  static const Eigen::Vector4f kGreen;
  static const Eigen::Vector4f kRed;
  static const Eigen::Vector4f kBlue;
  static const Eigen::Vector4f kGray;
  static const Eigen::Vector4f kBlack;
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

  static Eigen::Vector4f get(AgentId agent_id, float alpha) {
    if (color_map.find(agent_id) == color_map.end()) {
      color_map[agent_id] = random();
    }
    auto color = color_map[agent_id];
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
    std::string app_name{"ARIA Visualizer"};

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
  void setTime(std::time_t timestamp = std::time(nullptr)) {
    // get the nsec from the timestamp
    setTimeNSec(timestamp);
  }

  void toggleStepByStep() { params_.step_by_step = !params_.step_by_step; }

  void drawLines(const std::string& entity_path,
                 const std::vector<std::pair<Point3, Point3>>& points_pairs,
                 const std::vector<Eigen::Vector4f>& rgba,
                 float radius,
                 const std::vector<std::string>& labels = {},
                 const std::vector<std::string>& text = {}) {
    if (points_pairs.empty()) {
      return;
    }
    auto colors = rgba;
    if (rgba.size() == 1) {
      colors.resize(points_pairs.size(), rgba[0]);
    } else {
      CHECK(colors.size() == points_pairs.size(),
            fmt::format("{} colors.size() != points_pairs.size() {}",
                        colors.size(),
                        points_pairs.size()));
    }

    drawLinesImpl(entity_path, points_pairs, colors, radius, labels, text);
  }

  virtual void drawScalar(const std::string& entity_path, double value) {}

  virtual void drawLinesImpl(
      const std::string& entity_path,
      const std::vector<std::pair<Point3, Point3>>& points_pairs,
      const std::vector<Eigen::Vector4f>& rgba,
      float radius,
      const std::vector<std::string>& labels,
      const std::vector<std::string>& text) {}

  void drawPoints(const std::string& entity_path,
                  const std::vector<Point3>& points,
                  const Eigen::Vector4f& rgba,
                  float radius,
                  bool is_static = false) {
    std::vector<Eigen::Vector4f> rgbs(points.size(), rgba);
    drawPoints(entity_path, points, rgbs, {radius}, {}, is_static);
  }

  void drawPoints(const std::string& entity_path,
                  const std::vector<Point3>& points,
                  const std::vector<Eigen::Vector4f>& rgba,
                  const std::vector<float>& radius,
                  const std::vector<std::string>& labels,
                  bool is_static = false) {
    drawPointsImpl(entity_path, points, rgba, radius, labels, is_static);
  }

  virtual void drawPointsImpl(const std::string& entity_path,
                              const std::vector<Point3>& points,
                              const std::vector<Eigen::Vector4f>& rgba,
                              const std::vector<float>& radius,
                              const std::vector<std::string>& labels,
                              bool is_static = false) {}

  void drawPoints(const std::string& entity_path,
                  const std::vector<Point3>& points,
                  const Eigen::Vector4f& rgba,
                  const std::vector<float>& radius,
                  const std::vector<std::string>& labels,
                  bool is_static = false) {
    std::vector<Eigen::Vector4f> rgbs(points.size(), rgba);
    drawPointsImpl(entity_path, points, rgbs, radius, labels, is_static);
  }

  void drawPoints(const std::string& entity_path,
                  const Values& values,
                  const std::vector<Eigen::Vector4f>& rgba,
                  std::vector<float> radius,
                  std::vector<std::string> labels = {},
                  bool is_static = false);

  static std::optional<Point3> getPoint3(const Key& key, const Values& values) {
    if (values.exists(key) == false) {
      return std::nullopt;
    }
    auto dim = values.at(key).dim();
    if (dim == 3) {
      try {
        auto point = values.at<Pose2>(key);
        return Point3(point.x(), point.y(), 0);
      } catch (gtsam::ValuesIncorrectType& e) {
        auto point = values.at<Point3>(key);
        return point;
      }
    } else if (dim == 6) {
      auto point = values.at<Pose3>(key);
      return Pose3(point).translation();
    } else {
      return std::nullopt;
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
                     bool is_static = false,
                     float height = 10.) {
    std::vector<std::pair<Point3, Point3>> points;
    for (auto key : keys) {
      auto point = getPoint3(key, values);
      if (point) {
        Point3 p_up{point->x(), point->y(), point->z() + radius * height};
        points.emplace_back(*point, p_up);
      }
    }

    std::vector<Eigen::Vector4f> rgbs(points.size(), rgba);
    drawLines(entity_path, points, rgbs, radius);
  }

  template <typename ContainerT>
  void visualizeGTCameraPoses(const std::string& entity_path,
                              const ContainerT& frames);

  static std::vector<double> getEllipseFromCov(const Eigen::Matrix2d& cov);
  static std::vector<double> getEllipseFromCov(const Eigen::Matrix3d& cov);

  void drawUncertainty(const std::string& entity_path,
                       const Pose2& mean,
                       const Eigen::Matrix2d& cov,
                       const Eigen::Vector4f& rgba,
                       float line_width,
                       bool is_static = false) {
    drawUncertainty(
        entity_path, mean.translation(), cov, rgba, line_width, is_static);
  }

  void drawUncertainty(const std::string& entity_path,
                       const Point2& mean,
                       const Eigen::Matrix2d& cov,
                       const Eigen::Vector4f& rgba,
                       float line_width,
                       bool is_static = false) {
    drawUncertaintyImpl2D(
        entity_path, mean, getEllipseFromCov(cov), rgba, line_width, is_static);
  }

  void drawUncertainty(const std::string& entity_path,
                       const Pose3& mean,
                       const Eigen::Matrix3d& cov,
                       const Eigen::Vector4f& rgba,
                       float line_width,
                       bool is_static = false) {
    drawUncertainty(
        entity_path, mean.translation(), cov, rgba, line_width, is_static);
  }

  void drawUncertainty(const std::string& entity_path,
                       const Point3& mean,
                       const Eigen::Matrix3d& cov,
                       const Eigen::Vector4f& rgba,
                       float line_width,
                       bool is_static = false) {
    drawUncertaintyImpl3D(
        entity_path, mean, getEllipseFromCov(cov), rgba, line_width, is_static);
  }

  void drawUncertainty(const std::string& entity_path,
                       const std::vector<Point3>& mean,
                       const std::vector<Eigen::Matrix3d>& cov,
                       const Eigen::Vector4f& rgba,
                       float line_width,
                       bool is_static = false) {
    for (size_t i = 0; i < mean.size(); i++) {
      drawUncertainty(entity_path + "/" + std::to_string(i),
                      mean[i],
                      cov[i],
                      rgba,
                      line_width,
                      is_static);
    }
  }

  virtual void drawUncertaintyImpl2D(const std::string& entity_path,
                                     const Point2& mean,
                                     const std::vector<double>& ellipse,
                                     const Eigen::Vector4f& rgba,
                                     float line_width,
                                     bool is_static) {}

  virtual void drawUncertaintyImpl3D(const std::string& entity_path,
                                     const Point3& mean,
                                     const std::vector<double>& ellipse,
                                     const Eigen::Vector4f& rgba,
                                     float line_width,
                                     bool is_static) {}

  //  void drawBayesTree(const std::string& entity_path,

  template <typename FactorType>
  void drawFactors(const std::string& entity_path,
                   const FactorGraph<FactorType>& factors,
                   const Values& values,
                   const Eigen::Vector4f& rgba,
                   float line_width,
                   bool show_labels = false) {
    std::vector<Eigen::Vector4f> colors(factors.size(), rgba);
    drawFactors(entity_path, factors, values, colors, line_width, show_labels);
  }

  template <typename FactorType>
  void drawFactors(const std::string& entity_path,
                   const FactorGraph<FactorType>& factors,
                   const Values& values,
                   const std::vector<Eigen::Vector4f>& rgba,
                   float line_width,
                   bool show_labels = false) {
    std::vector<std::pair<Point3, Point3>> points;
    std::vector<std::string> labels;
    std::vector<Eigen::Vector4f> colors;
    for (size_t i = 0; i < factors.size(); i++) {
      auto factor = factors.at(i);
      if (factor == nullptr) {
        continue;
      }

      colors.push_back(rgba.at(i));
      auto keys = factor->keys();
      CHECK(keys.size() <= 2,
            "Not implemented for factors with more than 2 keys");

      auto key = keys[0];
      std::optional<Point3> p0, p1;
      if (keys.size() == 1) {
        p0 = getPoint3(key, values);
        if (p0.has_value()) p1 = *p0 + Point3(0, 0, 1.0);
      } else {
        p0 = getPoint3(keys[0], values);
        p1 = getPoint3(keys[1], values);
      }

      if (p0.has_value() && p1.has_value()) {
        points.emplace_back(*p0, *p1);
        if (keys.size() == 2) {
          labels.push_back(fmt::format(
              "{}-{}", DefaultKeyFormatter(key), DefaultKeyFormatter(keys[1])));
        } else {
          labels.push_back(fmt::format("{}", DefaultKeyFormatter(key)));
        }
      }
    }

    drawLines(entity_path,
              points,
              colors,
              line_width,
              show_labels ? labels : std::vector<std::string>{});
  }

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
