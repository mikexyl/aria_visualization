#include "aria_viz/visualizer_rerun.h"

#include <aria_common/benchmark.h>
#include <aria_common/logging.h>
#include <gtsam/inference/VariableIndex.h>

#include <opencv2/imgproc.hpp>

using namespace gtsam;

namespace aria::viz {

void VisualizerRerun::setTimeNSec(size_t timestamp) {
  #ifndef RERUN_SDK_OLD_VERSION
  rec_->set_time_timestamp_nanos_since_epoch("time", timestamp);
  #else
  rec_->set_time_nanos("time", timestamp);
  #endif
}

void VisualizerRerun::connectPositions3D(
    const std::string& entity_path,
    const std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>& positions,
    const std::vector<Eigen::Vector4f>& rgba,
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
                .with_colors(fromEigen(rgba))
                .with_radii({rerun::components::Radius::ui_points(radius)})
                .with_labels(labels));
}

void VisualizerRerun::drawLinesImpl(
    const std::string& entity_path,
    const std::vector<std::pair<Point3, Point3>>& points_pairs,
    const std::vector<Eigen::Vector4f>& rgba,
    float radius,
    const std::vector<std::string>& labels,
    const std::vector<std::string>& text) {
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [point0, point1] : points_pairs) {
    positions.push_back({point0.cast<float>(), point1.cast<float>()});
  }

  connectPositions3D(entity_path, positions, rgba, radius, labels, false, text);
}

void VisualizerRerun::drawPointsImpl(const std::string& entity_path,
                                     const std::vector<Point3>& points,
                                     const std::vector<Eigen::Vector4f>& rgba,
                                     const std::vector<float>& radius,
                                     const std::vector<std::string>& labels,
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

  rerun::Collection<rerun::components::Radius> radii;
  for (auto r : radius) {
    radii.take_ownership(rerun::components::Radius::ui_points(r));
  }

  rec_->log_with_static(entity_path,
                        is_static,
                        rerun::Points3D(points_eigen)
                            .with_colors(colors)
                            .with_radii(radii)
                            .with_labels(labels));
}

void VisualizerRerun::drawUncertaintyImpl2D(const std::string& entity_path,
                                            const Point2& mean,
                                            const std::vector<double>& ellipse,
                                            const Eigen::Vector4f& rgba,
                                            float line_width,
                                            bool is_static) {
  double width = ellipse[0], height = ellipse[1], angle = ellipse[5];
  rec_->log(entity_path,
            rerun::Ellipsoids3D::from_centers_and_radii(
                {{mean.x(), mean.y(), 0}}, {{width, height, 0}})
                .with_colors({fromEigen(rgba)})
                .with_rotation_axis_angles({rerun::RotationAxisAngle(
                    {0, 0, 1}, rerun::Angle::radians(angle))})
                .with_line_radii(line_width));
}

void VisualizerRerun::drawUncertaintyImpl3D(const std::string& entity_path,
                                            const Point3& mean,
                                            const std::vector<double>& ellipse,
                                            const Eigen::Vector4f& rgba,
                                            float line_width,
                                            bool is_static) {
  rec_->log_with_static(
      entity_path,
      is_static,
      rerun::Ellipsoids3D::from_centers_and_radii(
          {{mean.x(), mean.y(), mean.z()}},
          {{ellipse[0], ellipse[1], ellipse[2]}})
          .with_colors({fromEigen(rgba)})
          .with_rotation_axis_angles(
              {rerun::RotationAxisAngle({1, 0, 0},
                                        rerun::Angle::radians(ellipse[3])),
               rerun::RotationAxisAngle({0, 1, 0},
                                        rerun::Angle::radians(ellipse[4])),
               rerun::RotationAxisAngle({0, 0, 1},
                                        rerun::Angle::radians(ellipse[5]))})
          .with_line_radii(rerun::components::Radius::ui_points(line_width)));
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

    #ifndef RERUN_SDK_OLD_VERSION
    rec_->log_static(
        "timing/mean/" + label_with_index,
        rerun::SeriesLines().with_colors({{color(0), color(1), color(2)}}));
    rec_->log_static(
        "timing/total/" + label_with_index,
        rerun::SeriesLines().with_colors({{color(0), color(1), color(2)}}));
    #endif

    // Log the stats
    rec_->log("timing/mean/" + label_with_index,
              rerun::Scalar(stat.mean));
    rec_->log("timing/total/" + label_with_index,
              rerun::Scalar(stat.mean * stat.count));
  }
}

void VisualizerRerun::drawBayesTreeEdges(
    const std::string& entity_path,
    std::vector<std::pair<GaussianBayesTreeClique::shared_ptr,
                          GaussianBayesTreeClique::shared_ptr>> edges,
    std::vector<Eigen::Vector4f> rgba,
    float line_width,
    bool is_static) {
  std::vector<rerun::components::GraphEdge> rerun_edges;
  std::vector<std::string> cliques;
  for (size_t i = 0; i < edges.size(); i++) {
    auto [clique0, clique1] = edges[i];
    rerun_edges.push_back(
        {fmt::format("{}", *clique0), fmt::format("{}", *clique1)});
    cliques.push_back(fmt::format("{}", *clique0));
    cliques.push_back(fmt::format("{}", *clique1));
  }

  std::vector<rerun::components::Color> colors;
  for (const auto& color : rgba) {
    colors.push_back(fromEigen(color));
  }

  // log nodes with blue color
  std::vector<rerun::components::Color> clique_colors;
  if (rgba.size() == 1) {
    clique_colors.resize(cliques.size(), fromEigen(rgba[0]));
  } else {
    LOG_FATAL("multiple colors not supported for now");
  }

  rec_->log_with_static(
      entity_path,
      is_static,
      rerun::GraphNodes(cliques).with_labels(cliques).with_colors(
          clique_colors));

  //! rerun doesn't support colored edges
  // rec_->log_with_static(
  //     entity_path,
  //     is_static,
  //     rerun::GraphEdges(rerun_edges)
  //         .with_line_radii(rerun::components::Radius::ui_points(line_width))
  //         .with_graph_type(rerun::components::GraphType::Directed));
}

}  // namespace aria::viz