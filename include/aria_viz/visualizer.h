#pragma once

#include <aria_common/logging.h>
#include <aria_common/macros.h>
#include <aria_common/types.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <spdlog/fmt/fmt.h>

#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using namespace gtsam;

namespace aria::viz {

using MeshTriangle = std::array<uint32_t, 3>;
using MeshTexcoord = std::array<float, 2>;

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
    std::uniform_int_distribution<> dis(0, 255);

    // generate a random vector from 0 to 256
    float r = dis(gen);
    float g = dis(gen);
    float b = dis(gen);
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
  static const Eigen::Vector4f kLightBlue;
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

  static Eigen::Vector4f get(AgentId agent_id, float alpha = 255) {
    int agent_id_int = static_cast<int>(agent_id - 'a');
    if (color_map.find(agent_id_int) == color_map.end()) {
      color_map[agent_id_int] = random();
    }
    auto color = color_map[agent_id_int];
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

  virtual void plotBenchmarkStats() {}

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

  void drawMesh(const std::string& entity_path,
                const std::vector<Point3>& vertex_positions,
                const std::vector<MeshTriangle>& triangle_indices,
                const std::vector<Eigen::Vector4f>& vertex_colors = {},
                const std::vector<Point3>& vertex_normals = {},
                bool is_static = false) {
    drawMeshImpl(entity_path,
                 vertex_positions,
                 triangle_indices,
                 vertex_colors,
                 vertex_normals,
                 is_static);
  }

  virtual void drawMeshImpl(const std::string& entity_path,
                            const std::vector<Point3>& vertex_positions,
                            const std::vector<MeshTriangle>& triangle_indices,
                            const std::vector<Eigen::Vector4f>& vertex_colors,
                            const std::vector<Point3>& vertex_normals,
                            bool is_static = false) {}

  void drawTexturedMesh(const std::string& entity_path,
                        const std::vector<Point3>& vertex_positions,
                        const std::vector<MeshTriangle>& triangle_indices,
                        const std::vector<MeshTexcoord>& vertex_texcoords,
                        const cv::Mat& albedo_texture,
                        const std::vector<Eigen::Vector4f>& vertex_colors = {},
                        const std::vector<Point3>& vertex_normals = {},
                        bool is_static = false) {
    drawTexturedMeshImpl(entity_path,
                         vertex_positions,
                         triangle_indices,
                         vertex_texcoords,
                         albedo_texture,
                         vertex_colors,
                         vertex_normals,
                         is_static);
  }

  virtual void drawTexturedMeshImpl(
      const std::string& entity_path,
      const std::vector<Point3>& vertex_positions,
      const std::vector<MeshTriangle>& triangle_indices,
      const std::vector<MeshTexcoord>& vertex_texcoords,
      const cv::Mat& albedo_texture,
      const std::vector<Eigen::Vector4f>& vertex_colors,
      const std::vector<Point3>& vertex_normals,
      bool is_static = false) {
    (void)vertex_texcoords;
    (void)albedo_texture;
    drawMeshImpl(entity_path,
                 vertex_positions,
                 triangle_indices,
                 vertex_colors,
                 vertex_normals,
                 is_static);
  }

  void drawMeshFile(const std::string& entity_path,
                    const std::filesystem::path& mesh_path,
                    bool is_static = false) {
    drawMeshFileImpl(entity_path, mesh_path, is_static);
  }

  virtual void drawMeshFileImpl(const std::string& entity_path,
                                const std::filesystem::path& mesh_path,
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
    const Value& value = values.at(key);
    if (typeid(value) == typeid(GenericValue<Point3>)) {
      return values.at<Point3>(key);
    } else if (typeid(value) == typeid(GenericValue<Pose3>)) {
      return values.at<Pose3>(key).translation();
    } else if (typeid(value) == typeid(GenericValue<Point2>)) {
      const Point2& p2 = values.at<Point2>(key);
      return Point3(p2.x(), p2.y(), 0.0);
    } else if (typeid(value) == typeid(GenericValue<Pose2>)) {
      const Pose2& p2 = values.at<Pose2>(key);
      return Point3(p2.x(), p2.y(), 0.0);
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
                   bool show_labels = false,
                   bool ignore_missing_values = false,
                   const KeyFormatter& key_formatter = DefaultKeyFormatter) {
    std::vector<Eigen::Vector4f> colors(factors.size(), rgba);
    drawFactors(entity_path,
                factors,
                values,
                colors,
                line_width,
                show_labels,
                ignore_missing_values,
                key_formatter);
  }

  template <typename FactorType>
  void drawFactors(const std::string& entity_path,
                   const FactorGraph<FactorType>& factors,
                   const Values& values,
                   const std::vector<Eigen::Vector4f>& rgba,
                   float line_width,
                   bool show_labels = false,
                   bool ignore_missing_values = false,
                   const KeyFormatter& key_formatter = DefaultKeyFormatter) {
    std::vector<std::pair<Point3, Point3>> points;
    std::vector<std::string> labels;
    std::vector<Eigen::Vector4f> colors;
    for (size_t i = 0; i < factors.size(); i++) {
      auto factor = factors.at(i);
      if (factor == nullptr) {
        continue;
      }

      auto keys = factor->keys();
      if (keys.size() > 2) {
        // spdlog::warn("Factor has more than 2 keys, skip");
        continue;
      }

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
          labels.push_back(
              fmt::format("{}-{}", key_formatter(key), key_formatter(keys[1])));
        } else {
          labels.push_back(fmt::format("{}", key_formatter(key)));
        }
        colors.push_back(rgba.at(i));
      } else {
        if (ignore_missing_values) {
          continue;
        }
        LOG_FATAL("Factor has no value for key {} or {}",
                  key_formatter(keys[0]),
                  key_formatter(keys[1]));
      }
    }

    drawLines(entity_path,
              points,
              colors,
              line_width,
              show_labels ? labels : std::vector<std::string>{});
  }

  void drawTf(const std::string& entity_path,
              Pose3 tf,
              float axis_length = 1.f,
              bool is_static = false) {
    drawTfImpl(entity_path, tf, axis_length, is_static);
  }

  void drawTrajectory(const std::string& entity_path,
                      const std::vector<Pose3>& poses,
                      const Eigen::Vector4f& rgba = ColorMap::kGreen,
                      float line_width = 0.5f,
                      bool is_static = false) {
    Values values;
    for (size_t i = 0; i < poses.size(); ++i) {
      values.insert(i, poses[i]);
    }

    // use symbolic factor graph to draw the trajectory
    gtsam::SymbolicFactorGraph::shared_ptr graph(new SymbolicFactorGraph());
    for (size_t i = 0; i < poses.size() - 1; ++i) {
      graph->add(gtsam::SymbolicFactor(i, i + 1));
    }

    if (is_static) {
      spdlog::warn("Drawing trajectory as static is not supported yet");
    }

    drawFactors(entity_path, *graph, values, {rgba}, line_width, false, true);
  }

  void drawLandmarks(const std::string& entity_path,
                     const std::vector<Point3>& landmarks,
                     const std::vector<long>& ids,
                     const std::vector<Eigen::Vector4f>& rgba,
                     const std::vector<float>& radius,
                     const std::vector<std::string>& labels = {},
                     bool is_static = false) {
    CHECK(ids.size() == landmarks.size() or ids.size() == 0,
          fmt::format("ids.size() != landmarks.size(), either empty. {} != {}",
                      ids.size(),
                      landmarks.size()));
    CHECK(rgba.size() == landmarks.size() or rgba.size() == 1,
          fmt::format(
              "rgba.size() != landmarks.size(), either single value. {} != {}",
              rgba.size(),
              landmarks.size()));

    std::vector<Eigen::Vector4f> colors;
    if (rgba.size() == 1) {
      colors.resize(landmarks.size(), rgba[0]);
    } else {
      colors = rgba;
    }

    std::vector<double> radii;
    if (radius.empty()) {
      radii.resize(landmarks.size(), 0.1f);
    } else if (radius.size() == 1) {
      radii.resize(landmarks.size(), radius[0]);
    } else {
      LOG_FATAL(
          "radius.size() != landmarks.size(), either single value. {} != {}",
          radius.size(),
          landmarks.size());
    }

    if (ids.empty()) {
      // if ids are empty, draw points as a single entity
      drawPoints(entity_path,
                 landmarks,
                 rgba.empty() ? ColorMap::kGray : rgba[0],
                 radius.empty() ? 0.1f : radius[0],
                 is_static);
    } else {
      // if ids are not empty, draw points in each entity path
      for (size_t i = 0; i < landmarks.size(); ++i) {
        std::string entity_path_with_id =
            fmt::format("{}/{}", entity_path, ids[i]);
        drawPoints(entity_path_with_id,
                   {landmarks.at(i)},
                   colors.at(i),
                   {radii.at(i)},
                   {},
                   is_static);
      }
    }
  }

 protected:
  virtual void step() {
    // set a keyboard callback for R
    int key = cv::waitKey(params_.step_by_step ? 0 : 1) & 0xFF;
    if (key == 'r') {
      toggleStepByStep();
    }
  }

  virtual void drawTfImpl(const std::string& entity_path,
                          const Pose3& tf,
                          float axis_length = 1.f,
                          bool is_static = false) {
    // default implementation does nothing
  }

 protected:
  Params params_;
};
}  // namespace aria::viz
