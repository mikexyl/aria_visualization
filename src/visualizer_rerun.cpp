#include "aria_viz/visualizer_rerun.h"

#include <aria_common/benchmark.h>
#include <aria_common/logging.h>
#include <gtsam/inference/VariableIndex.h>

#include <opencv2/imgproc.hpp>

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
    bool clear,
    const std::vector<std::string>& text) {
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
    const std::vector<std::string>& labels,
    const std::vector<std::string>& text) {
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [point0, point1] : points_pairs) {
    positions.push_back({point0.cast<float>(), point1.cast<float>()});
  }

  connectPositions3D(entity_path, positions, rgba, radius, labels, false, text);
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
  // rerun supports Ellipsoids3D from 0.18.x
  // https://rerun.io/docs/reference/types/archetypes/ellipsoids3d

  auto [width, height, angle] = getEllipseFromCov(cov);

  rec_->log(entity_path,
            rerun::Ellipsoids3D::from_centers_and_radii(
                {{mean.x(), mean.y(), 0}}, {{width, height, 0}})
                .with_colors({fromEigen(rgba)})
                .with_rotation_axis_angles({rerun::RotationAxisAngle(
                    {0, 0, 1}, rerun::Angle::radians(angle))})
                .with_line_radii(line_width));
}

void VisualizerRerun::visualizeUncertainty(const std::string& entity_path,
                                           const Point3& mean,
                                           const Eigen::Matrix3d& cov,
                                           const Eigen::Vector4f& rgba,
                                           float line_width) {
  auto ellipse = getEllipseFromCov(cov);

  rec_->log(entity_path,
            rerun::Ellipsoids3D::from_centers_and_radii(
                {{mean.x(), mean.y(), mean.z()}},
                {{ellipse[0], ellipse[1], ellipse[2]}})
                .with_colors({fromEigen(rgba)})
                .with_rotation_axis_angles(
                    {rerun::RotationAxisAngle(
                         {1, 0, 0}, rerun::Angle::radians(ellipse[3])),
                     rerun::RotationAxisAngle(
                         {0, 1, 0}, rerun::Angle::radians(ellipse[4])),
                     rerun::RotationAxisAngle(
                         {0, 0, 1}, rerun::Angle::radians(ellipse[5]))})
                .with_line_radii(line_width));
}

void VisualizerRerun::visualizeFactors(const std::string& entity_path,
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
      if (keys.size() == 2) {
        labels.push_back(fmt::format(
            "{}-{}", DefaultKeyFormatter(key), DefaultKeyFormatter(keys[1])));
      } else {
        labels.push_back(fmt::format("{}", DefaultKeyFormatter(key)));
      }
    }
  }

  connectPointsToPoints(entity_path,
                        points,
                        rgba,
                        line_width,
                        show_labels ? labels : std::vector<std::string>{});
}

std::tuple<double, double, double> VisualizerRerun::getEllipseFromCov(
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
  double angle_rad = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));

  return {width, height, angle_rad};
}

std::vector<double> VisualizerRerun::getEllipseFromCov(
    const Eigen::Matrix3d& cov) {
  // Compute the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
  if (eigensolver.info() != Eigen::Success) {
    std::cerr << "Failed to compute eigenvalues and eigenvectors." << std::endl;
    return {};
  }

  // Eigenvalues are the lengths of the ellipse's axes
  Eigen::Vector3d eigenvalues = eigensolver.eigenvalues();
  double x = std::sqrt(eigenvalues(0)) * 2;
  double y = std::sqrt(eigenvalues(1)) * 2;
  double z = std::sqrt(eigenvalues(2)) * 2;

  // Eigenvectors are the directions of the ellipse's axes
  Eigen::Matrix3d eigenvectors = eigensolver.eigenvectors();
  double angle_x_rad = std::atan2(eigenvectors(1, 0), eigenvectors(0, 0));
  double angle_y_rad = std::atan2(eigenvectors(2, 1), eigenvectors(1, 1));
  double angle_z_rad = std::atan2(eigenvectors(0, 2), eigenvectors(1, 2));

  return {x, y, z, angle_x_rad, angle_y_rad, angle_z_rad};
}

std::vector<Point3> VisualizerRerun::generateEllipse(
    const Eigen::Vector2d& mean,
    const Eigen::Matrix2d& cov) {
  auto [width, height, angle] = getEllipseFromCov(cov);
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

void VisualizerRerun::addSpdlogToRerun(spdlog::level::level_enum level) {
  // Ensure rec_ is valid
  if (!rec_) {
    throw std::runtime_error("RecordingStream pointer is null");
  }

  // Create a lambda function that captures `rec_` and adds messages to it
  auto rerun_logger = [this](const spdlog::details::log_msg& msg) {
    // Convert spdlog message to string, assuming msg.payload contains the log
    // message
    std::string message(msg.payload.begin(), msg.payload.end());

    // spdlog level convert to rerun level
    auto spdlog_level = msg.level;
    rerun::TextLogLevel level(spdlog::level::to_short_c_str(spdlog_level));

    // Forward the message to the RecordingStream
    rec_->log("spdlog", rerun::TextLog(message).with_level(level));
  };

  // Create a spdlog sink with the lambda callback
  auto sink = std::make_shared<logging::callback_sink_st>(rerun_logger);

  // Set the sink's log level
  sink->set_level(level);

  // Get the default logger instance and attach the new sink
  auto logger = spdlog::default_logger();
  if (!logger) {
    throw std::runtime_error("Default logger not found");
  }

  // Add the sink to the default logger
  logger->sinks().push_back(sink);
}

void VisualizerRerun::plotBenchmarkStats() {
  // Ensure rec_ is valid
  if (!rec_) {
    throw std::runtime_error("RecordingStream pointer is null");
  }

  // Get the benchmark stats

  // Iterate over the stats and log them
  for (auto& [label, stat] : benchmarkStatsMap) {
    std::lock_guard<std::mutex> lock(stat.mutex);

    auto color = ColorMap::random(stat.label);

    std::string label_with_index = stat.label;
    if (stat.index.has_value()) {
      label_with_index += "_" + std::to_string(stat.index.value());
    }

    rec_->log_static(
        "timing/" + label_with_index,
        rerun::SeriesLine().with_color({color(0), color(1), color(2)}));

    // Log the stats
    rec_->log("timing/" + label_with_index, rerun::Scalar(stat.mean));
  }
}

void VisualizerRerun::visualizeUncertainty2D(
    const std::string& entity_path,
    const std::vector<Point2>& mean,
    const std::vector<Eigen::Matrix2d>& cov,
    const Eigen::Vector4f& rgba,
    bool is_static) {
  static constexpr int kPlotWidth = 1080;
  static constexpr int kPlotHeight = 720;

  if (mean.empty() or cov.empty()) {
    return;
  }

  // Find min and max points for x and y
  auto [min_x_it, max_x_it] = std::minmax_element(
      mean.begin(), mean.end(), [](const Point2& a, const Point2& b) {
        return a.x() < b.x();
      });
  auto [min_y_it, max_y_it] = std::minmax_element(
      mean.begin(), mean.end(), [](const Point2& a, const Point2& b) {
        return a.y() < b.y();
      });

  int min_x = static_cast<int>(min_x_it->x());
  int max_x = static_cast<int>(max_x_it->x());
  int min_y = static_cast<int>(min_y_it->y());
  int max_y = static_cast<int>(max_y_it->y());

  float ratio = std::min(static_cast<float>(kPlotWidth) / (max_x - min_x),
                         static_cast<float>(kPlotHeight) / (max_y - min_y)) *
                0.6;

  cv::Mat img = cv::Mat::zeros(kPlotHeight, kPlotWidth, CV_8UC4);
  std::vector<Point2> ellipse_points;
  for (size_t i = 0; i < mean.size(); i++) {
    auto [width, height, angle_rad] = getEllipseFromCov(cov[i]);
    if (width <= 0 or height <= 0) {
      continue;
    }
    try {
      cv::ellipse(img,
                  cv::Point2f((mean[i].x() - min_x) * ratio + kPlotWidth * 0.2,
                              kPlotHeight - (mean[i].y() - min_y) * ratio -
                                  kPlotHeight * 0.2),
                  cv::Size(width * ratio, height * ratio),
                  angle_rad * 180.0 / M_PI,
                  0,
                  360,
                  cv::Scalar(rgba[0] * 255, rgba[1] * 255, rgba[2] * 255, 255),
                  1);
    } catch (cv::Exception& e) {
      spdlog::warn("VIZ: Failed to draw ellipse: {}", e.what());
    }
  }

  // publish the image to rerun
  rec_->log_with_static(
      entity_path,
      is_static,
      rerun::Image::from_rgba32(img, {kPlotWidth, kPlotHeight}));
}

void VisualizerRerun::visualizeUncertainty2D(
    const std::string& entity_path,
    const std::vector<Point3>& mean,
    const std::vector<Eigen::Matrix3d>& cov,
    const Eigen::Vector4f& rgba,
    bool is_static) {
  std::vector<Point2> points;
  std::vector<Eigen::Matrix2d> cov2d;
  for (size_t i = 0; i < mean.size(); i++) {
    points.push_back(Point2(mean[i].x(), mean[i].y()));
    cov2d.push_back(cov[i].block<2, 2>(0, 0));
  }

  visualizeUncertainty2D(entity_path, points, cov2d, rgba, is_static);
}

void VisualizerRerun::visualizeUncertainty2D(
    const std::string& entity_path,
    const NonlinearFactorGraph& factors,
    const Values& values,
    const Eigen::Vector4f& rgba,
    bool is_static) {
  std::map<Key, Point2> points;
  std::map<Key, Eigen::Matrix2d> cov;
  VariableIndex vi(factors);
  KeySet keys = factors.keys();
  for (Key key : keys) {
    auto vi_idx = vi.find(key);
    if (vi_idx == vi.end()) {
      continue;
    }

    auto pose = values.at<Pose3>(key);
    points[key] = Point2(pose.x(), pose.y());

    // read marginals from the factor
    auto factor_idx = vi_idx->second;
    CHECK_MSG(factor_idx.size() == 2, factor_idx.size());
    CHECK(factor_idx.front() == factor_idx.back());
    auto factor = factors.at(factor_idx[0]);
    auto noise_factor = boost::dynamic_pointer_cast<NoiseModelFactor>(factor);
    CHECK(noise_factor);

    auto noise = noise_factor->noiseModel();
    auto gaussian = boost::dynamic_pointer_cast<noiseModel::Gaussian>(noise);
    CHECK(gaussian);

    auto covariance = gaussian->covariance();
    cov[key] = covariance.block<2, 2>(0, 0);
  }

  std::vector<Point2> points_vec;
  std::vector<Eigen::Matrix2d> cov_vec;
  for (const auto& [key, point] : points) {
    points_vec.push_back(point);
    cov_vec.push_back(cov[key]);
  }

  visualizeUncertainty2D(entity_path, points_vec, cov_vec, rgba, is_static);
}

}  // namespace aria::viz