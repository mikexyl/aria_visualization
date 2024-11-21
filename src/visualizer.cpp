#include "aria_viz/visualizer.h"

namespace aria::viz {
std::map<int, Eigen::Vector3f> AgentColorMap::color_map = {
    {0, Eigen::Vector3f(255, 0, 0)},
    {1, Eigen::Vector3f(0, 255, 0)},
    // make it cyan-ish to be more visible in black background
    {2, Eigen::Vector3f(100, 100, 255)},
    {3, Eigen::Vector3f(255, 255, 0)},
    {4, Eigen::Vector3f(255, 0, 255)},
    {5, Eigen::Vector3f(0, 255, 255)},
    {6, Eigen::Vector3f(63, 255, 128)},
    {7, Eigen::Vector3f(128, 0, 0)},
    {8, Eigen::Vector3f(0, 128, 0)},
    {9, Eigen::Vector3f(0, 0, 128)},
    {10, Eigen::Vector3f(128, 128, 0)},
    {11, Eigen::Vector3f(128, 0, 128)},
    {12, Eigen::Vector3f(0, 128, 128)},
    {13, Eigen::Vector3f(128, 128, 128)},
    {14, Eigen::Vector3f(64, 0, 0)},
    {15, Eigen::Vector3f(0, 64, 0)},
    {16, Eigen::Vector3f(0, 0, 64)},
    {17, Eigen::Vector3f(64, 64, 0)},
    {18, Eigen::Vector3f(64, 0, 64)},
    {19, Eigen::Vector3f(0, 64, 64)},
    {20, Eigen::Vector3f(64, 64, 64)}};

const Eigen::Vector3f ColorMap::kGreen = Eigen::Vector3f(0, 255, 0);
const Eigen::Vector3f ColorMap::kRed = Eigen::Vector3f(255, 0, 0);
const Eigen::Vector3f ColorMap::kBlue = Eigen::Vector3f(0, 0, 255);
const Eigen::Vector3f ColorMap::kGray = Eigen::Vector3f(128, 128, 128);
const Eigen::Vector3f ColorMap::kBlack = Eigen::Vector3f(0, 0, 0);

class Visualizer;

void Visualizer::drawFactors(const std::string& entity_path,
                             const NonlinearFactorGraph& factors,
                             const Values& values,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool show_labels) {
  std::vector<std::pair<Point3, Point3>> points;
  std::vector<std::string> labels;
  for (const auto& factor : factors) {
    if (factor == nullptr) {
      continue;
    }
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
            rgba,
            line_width,
            show_labels ? labels : std::vector<std::string>{});
}
void Visualizer::drawPoints(const std::string& entity_path,
                            const Values& values,
                            const std::vector<Eigen::Vector4f>& rgba,
                            std::vector<float> radius,
                            bool is_static) {
  std::vector<Point3> points;
  // convert all values to points
  for (const auto& [key, value] : values) {
    if (auto point = getPoint3(key, values)) {
      points.push_back(*point);
    }
  }

  std::vector<Eigen::Vector4f> rgba_full;
  if (rgba.size() == 1) {
    rgba_full.resize(points.size(), rgba[0]);
  } else {
    rgba_full = rgba;
  }

  std::vector<float> radius_full;
  if (radius.size() == 1) {
    radius_full.resize(points.size(), radius[0]);
  } else {
    radius_full = radius;
  }

  drawPointsImpl(entity_path, points, rgba_full, radius_full, is_static);
}

}  // namespace aria::viz