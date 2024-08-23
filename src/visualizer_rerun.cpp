#include "aria_viz/visualizer_rerun.h"

#include <aria_common/logging.h>

#include "collection_adapters.hpp"

using namespace gtsam;

namespace aria::viz {

static rerun::Collection<rerun::TensorDimension> tensor_shape(
    const cv::Mat& img) {
  return {img.rows, img.cols, img.channels()};
};

void VisualizerRerun::setTimeNSec(size_t timestamp) {
  rec_->set_time_nanos("time", timestamp);
}

void VisualizerRerun::connectPositions3D(
    const std::string& entity_path,
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& positions,
    const Eigen::Vector4f& rgba,
    float radius,
    const std::vector<std::string>& labels,
    bool clear) {
  if (clear) rec_->log(entity_path, rerun::Clear(false));

  std::vector<rerun::Collection<rerun::Vec3D>> lines;

  for (const auto& [pos0, pos1] : positions) {
    rerun::Vec3D p0(pos0.data());
    rerun::Vec3D p1(pos1.data());
    lines.push_back({p0, p1});
  }

  rec_->log(entity_path,
            rerun::LineStrips3D(lines)
                .with_colors({fromEigen(rgba)})
                .with_radii({radius})
                .with_labels(labels));
}

void VisualizerRerun::connectPointsToPoints(
    const std::string& entity_path,
    const std::vector<std::pair<Point3, Point3>>& points_pairs,
    Eigen::Vector4f rgba,
    float radius,
    const std::vector<std::string>& labels) {
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [point0, point1] : points_pairs) {
    positions.push_back({point0.cast<float>(), point1.cast<float>()});
  }

  connectPositions3D(entity_path, positions, rgba, radius, labels, false);
}

float VisualizerRerun::visualizePoints(const std::string& entity_path,
                                       const std::vector<Point3>& points,
                                       const std::vector<Eigen::Vector4f>& rgba,
                                       float radius,
                                       bool is_static) {
  if (radius < 0.0) {
    // when radius set negative, use adaptive radius
    float range_x[2] = {std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::min()};
    float range_y[2] = {std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::min()};
    float range_z[2] = {std::numeric_limits<float>::max(),
                        std::numeric_limits<float>::min()};

    for (const auto& point : points) {
      range_x[0] = std::fmin(range_x[0], point.x());
      range_x[1] = std::fmax(range_x[1], point.x());
      range_y[0] = std::fmin(range_y[0], point.y());
      range_y[1] = std::fmax(range_y[1], point.y());
      range_z[0] = std::fmin(range_z[0], point.z());
      range_z[1] = std::fmax(range_z[1], point.z());
    }

    float diff_x = range_x[1] - range_x[0], diff_y = range_y[1] - range_y[0],
          diff_z = range_z[1] - range_z[0];

    // use the max diff and times 1e-4
    radius = std::max({diff_x, diff_y, diff_z}) * 2e-3;
    radius = std::min(radius, 1.0f);
  }

  std::vector<float> radii(points.size(), radius);

  visualizePoints(entity_path, points, rgba, radii, is_static);

  return radius;
}

void VisualizerRerun::visualizePoints(const std::string& entity_path,
                                      const std::vector<Point3>& points,
                                      const std::vector<Eigen::Vector4f>& rgba,
                                      std::vector<float> radius,
                                      bool is_static) {
  std::vector<rerun::Color> colors;
  for (size_t i = 0; i < rgba.size(); i++) {
    auto color = fromEigen(rgba[i]);
    colors.push_back(color);
  }
  std::vector<Eigen::Vector3f> points_eigen;
  for (const auto& point : points) {
    points_eigen.push_back(point.cast<float>());
  }

  rec_->log_with_static(
      entity_path,
      is_static,
      rerun::Points3D(points_eigen).with_colors(colors).with_radii(radius));
}
void VisualizerRerun::visualizeUncertainty(const std::string& entity_path,
                                           const Point2& mean,
                                           const Eigen::Matrix2d& cov,
                                           const Eigen::Vector4f& rgba,
                                           float line_width) {
  auto ellipse_points =
      generateEllipse(mean, cov);  // only visualize the first one

  // convert ellipse points to point pairs
  std::vector<std::pair<Point3, Point3>> points_pairs;
  for (size_t j = 0; j < ellipse_points.size() - 1; j++) {
    points_pairs.push_back({ellipse_points[j], ellipse_points[j + 1]});
  }
  points_pairs.push_back({ellipse_points.back(), ellipse_points.front()});

  connectPointsToPoints(entity_path, points_pairs, rgba, line_width);
}

void VisualizerRerun::visualizeFactors(const std::string& entity_path,
                                       const NonlinearFactorGraph& factors,
                                       const Values& values,
                                       const Eigen::Vector4f& rgba,
                                       float line_width) {
  std::vector<std::pair<Point3, Point3>> points;
  for (const auto& factor : factors) {
    if (factor == nullptr) {
      continue;
    }
    auto keys = factor->keys();
    CHECK_MSG(keys.size() <= 2,
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
    }
  }

  connectPointsToPoints(entity_path, points, rgba, line_width);
}

std::vector<Point3> VisualizerRerun::generateEllipse(
    const Eigen::Vector2d& mean,
    const Eigen::Matrix2d& cov) {
  // Compute the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);
  if (eigensolver.info() != Eigen::Success) {
    std::cerr << "Failed to compute eigenvalues and eigenvectors." << std::endl;
    return {};
  }

  // Eigenvalues are the lengths of the ellipse's axes
  Eigen::Vector2d eigenvalues = eigensolver.eigenvalues();
  double width = std::sqrt(eigenvalues(0)) * 4;
  double height = std::sqrt(eigenvalues(1)) * 4;

  // Eigenvectors are the directions of the ellipse's axes
  Eigen::Matrix2d eigenvectors = eigensolver.eigenvectors();
  double angle = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));

  // Generate ellipse points
  std::vector<Point3> ellipse_points;
  int num_points = 100;
  for (int i = 0; i < num_points; ++i) {
    double theta = 2.0 * M_PI * i / num_points;
    double x_ = width * std::cos(theta) / 2.0;
    double y_ = height * std::sin(theta) / 2.0;

    // Rotate the points
    double x_rot = std::cos(angle) * x_ - std::sin(angle) * y_;
    double y_rot = std::sin(angle) * x_ + std::cos(angle) * y_;

    // Translate the points
    ellipse_points.push_back({x_rot + mean.x(), y_rot + mean.y(), 0});
  }
  return ellipse_points;
}

}  // namespace aria::viz