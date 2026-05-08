#include "aria_viz/visualizer_rerun.h"

#include <aria_common/benchmark.h>
#include <aria_common/logging.h>
#include <gtsam/inference/VariableIndex.h>

#include <opencv2/imgproc.hpp>

#include <cmath>

using namespace gtsam;

namespace aria::viz {

namespace {

std::vector<rerun::components::Position3D> toRerunPositions(
    const std::vector<Point3>& points) {
  std::vector<rerun::components::Position3D> positions;
  positions.reserve(points.size());
  for (const auto& point : points) {
    positions.emplace_back(point.x(), point.y(), point.z());
  }
  return positions;
}

std::vector<rerun::components::Vector3D> toRerunVectors(
    const std::vector<Point3>& points) {
  std::vector<rerun::components::Vector3D> vectors;
  vectors.reserve(points.size());
  for (const auto& point : points) {
    vectors.emplace_back(point.x(), point.y(), point.z());
  }
  return vectors;
}

std::vector<rerun::components::TriangleIndices> toRerunTriangles(
    const std::vector<MeshTriangle>& triangle_indices) {
  std::vector<rerun::components::TriangleIndices> triangles;
  triangles.reserve(triangle_indices.size());
  for (const auto& triangle : triangle_indices) {
    triangles.emplace_back(triangle);
  }
  return triangles;
}

std::vector<rerun::components::Color> toRerunColors(
    const std::vector<Eigen::Vector4f>& rgba) {
  std::vector<rerun::components::Color> colors;
  colors.reserve(rgba.size());
  for (const auto& color : rgba) {
    colors.push_back(fromEigen(color));
  }
  return colors;
}

std::vector<rerun::components::Texcoord2D> toRerunTexcoords(
    const std::vector<MeshTexcoord>& texcoords) {
  std::vector<rerun::components::Texcoord2D> rerun_texcoords;
  rerun_texcoords.reserve(texcoords.size());
  for (const auto& texcoord : texcoords) {
    rerun_texcoords.emplace_back(texcoord[0], texcoord[1]);
  }
  return rerun_texcoords;
}

bool isFinite(const Point3& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

bool isFinite(const Eigen::Vector4f& vector) {
  return vector.allFinite();
}

bool isValidEllipse(const std::vector<double>& ellipse, size_t expected_size) {
  if (ellipse.size() < expected_size) {
    return false;
  }
  for (size_t i = 0u; i < expected_size; ++i) {
    if (!std::isfinite(ellipse[i])) {
      return false;
    }
  }
  return true;
}

struct TextureImageData {
  rerun::components::ImageBuffer buffer;
  rerun::components::ImageFormat format;
};

TextureImageData toRerunTextureImage(const cv::Mat& texture_image) {
  if (texture_image.empty()) {
    throw std::runtime_error("Texture image is empty");
  }

  cv::Mat converted_texture;
  rerun::datatypes::ColorModel color_model;
  switch (texture_image.type()) {
    case CV_8UC1:
      cv::cvtColor(texture_image, converted_texture, cv::COLOR_GRAY2RGB);
      color_model = rerun::datatypes::ColorModel::RGB;
      break;
    case CV_8UC3:
      cv::cvtColor(texture_image, converted_texture, cv::COLOR_BGR2RGB);
      color_model = rerun::datatypes::ColorModel::RGB;
      break;
    case CV_8UC4:
      cv::cvtColor(texture_image, converted_texture, cv::COLOR_BGRA2RGBA);
      color_model = rerun::datatypes::ColorModel::RGBA;
      break;
    default:
      throw std::runtime_error("Unsupported texture image type");
  }

  if (!converted_texture.isContinuous()) {
    converted_texture = converted_texture.clone();
  }

  const auto num_bytes = converted_texture.total() * converted_texture.elemSize();
  std::vector<uint8_t> texture_bytes(converted_texture.data,
                                     converted_texture.data + num_bytes);

  TextureImageData texture_data;
  texture_data.buffer =
      rerun::components::ImageBuffer(rerun::take_ownership(std::move(texture_bytes)));
  texture_data.format = rerun::components::ImageFormat(
      rerun::WidthHeight(static_cast<uint32_t>(converted_texture.cols),
                         static_cast<uint32_t>(converted_texture.rows)),
      color_model,
      rerun::datatypes::ChannelDatatype::U8);
  return texture_data;
}

}  // namespace

void VisualizerRerun::setTimeNSec(size_t timestamp) {
  rec_->set_time_timestamp_nanos_since_epoch("time", timestamp);
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
    radii.take_ownership(rerun::components::Radius(r));
  }

  rec_->log_with_static(entity_path,
                        is_static,
                        rerun::Points3D(points_eigen)
                            .with_colors(colors)
                            .with_radii(radii)
                            .with_labels(labels));
}

void VisualizerRerun::drawMeshImpl(
    const std::string& entity_path,
    const std::vector<Point3>& vertex_positions,
    const std::vector<MeshTriangle>& triangle_indices,
    const std::vector<Eigen::Vector4f>& vertex_colors,
    const std::vector<Point3>& vertex_normals,
    bool is_static) {
  if (vertex_positions.empty()) {
    spdlog::warn("Skipping empty mesh for entity: {}", entity_path);
    return;
  }

  auto mesh = rerun::Mesh3D(toRerunPositions(vertex_positions));
  if (!triangle_indices.empty()) {
    mesh = std::move(mesh).with_triangle_indices(
        toRerunTriangles(triangle_indices));
  }
  if (!vertex_colors.empty()) {
    mesh = std::move(mesh).with_vertex_colors(toRerunColors(vertex_colors));
  }
  if (!vertex_normals.empty()) {
    mesh =
        std::move(mesh).with_vertex_normals(toRerunVectors(vertex_normals));
  }

  rec_->log_with_static(entity_path, is_static, std::move(mesh));
}

void VisualizerRerun::drawTexturedMeshImpl(
    const std::string& entity_path,
    const std::vector<Point3>& vertex_positions,
    const std::vector<MeshTriangle>& triangle_indices,
    const std::vector<MeshTexcoord>& vertex_texcoords,
    const cv::Mat& albedo_texture,
    const std::vector<Eigen::Vector4f>& vertex_colors,
    const std::vector<Point3>& vertex_normals,
    bool is_static) {
  if (vertex_positions.empty()) {
    spdlog::warn("Skipping empty textured mesh for entity: {}", entity_path);
    return;
  }
  if (vertex_texcoords.empty()) {
    throw std::runtime_error("Textured mesh requires per-vertex UVs");
  }
  if (vertex_positions.size() != vertex_texcoords.size()) {
    throw std::runtime_error(
        "Textured mesh requires one UV coordinate per vertex");
  }

  auto texture_image = toRerunTextureImage(albedo_texture);

  auto mesh = rerun::Mesh3D(toRerunPositions(vertex_positions))
                  .with_vertex_texcoords(toRerunTexcoords(vertex_texcoords))
                  .with_albedo_texture_buffer(texture_image.buffer)
                  .with_albedo_texture_format(texture_image.format);

  if (!triangle_indices.empty()) {
    mesh = std::move(mesh).with_triangle_indices(
        toRerunTriangles(triangle_indices));
  }
  if (!vertex_colors.empty()) {
    mesh = std::move(mesh).with_vertex_colors(toRerunColors(vertex_colors));
  }
  if (!vertex_normals.empty()) {
    mesh =
        std::move(mesh).with_vertex_normals(toRerunVectors(vertex_normals));
  }

  rec_->log_with_static(entity_path, is_static, std::move(mesh));
}

void VisualizerRerun::drawMeshFileImpl(const std::string& entity_path,
                                       const std::filesystem::path& mesh_path,
                                       bool is_static) {
  if (!std::filesystem::exists(mesh_path)) {
    throw std::runtime_error("Mesh file does not exist: " + mesh_path.string());
  }

  rec_->log_with_static(
      entity_path,
      is_static,
      rerun::Asset3D::from_file_path(mesh_path).value_or_throw());
}

void VisualizerRerun::drawUncertaintyImpl2D(const std::string& entity_path,
                                            const Point2& mean,
                                            const std::vector<double>& ellipse,
                                            const Eigen::Vector4f& rgba,
                                            float line_width,
                                            bool is_static) {
  if (!isValidEllipse(ellipse, 3u) || !isFinite(rgba)) {
    spdlog::warn("Skipping invalid 2D uncertainty for entity: {}", entity_path);
    return;
  }

  double width = ellipse[0], height = ellipse[1], angle = ellipse[2];
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
  if (!isFinite(mean) || !isValidEllipse(ellipse, 6u) || !isFinite(rgba)) {
    spdlog::warn("Skipping invalid 3D uncertainty for entity: {}", entity_path);
    return;
  }

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

    rec_->log_static(
        "timing/mean/" + label_with_index,
        rerun::SeriesLines().with_colors({{color(0), color(1), color(2)}}));
    rec_->log_static(
        "timing/total/" + label_with_index,
        rerun::SeriesLines().with_colors({{color(0), color(1), color(2)}}));

    // Log the stats
    rec_->log("timing/mean/" + label_with_index,
              rerun::Scalars(std::vector<double>{stat.mean}));
    rec_->log("timing/total/" + label_with_index,
              rerun::Scalars(std::vector<double>{stat.mean * stat.count}));
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

void VisualizerRerun::drawTfImpl(const std::string& entity_path,
                                 const Pose3& tf,
                                 float axis_length,
                                 bool is_static) {
  const Point3 translation(tf.x(), tf.y(), tf.z());
  const auto quaternion = tf.rotation().toQuaternion();
  const double quaternion_norm = quaternion.norm();
  if (!isFinite(translation) || !std::isfinite(quaternion.w()) ||
      !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
      !std::isfinite(quaternion.z()) || quaternion_norm <= 1e-12) {
    spdlog::warn("Skipping invalid transform for entity: {}", entity_path);
    return;
  }

  const float q_w = static_cast<float>(quaternion.w() / quaternion_norm);
  const float q_x = static_cast<float>(quaternion.x() / quaternion_norm);
  const float q_y = static_cast<float>(quaternion.y() / quaternion_norm);
  const float q_z = static_cast<float>(quaternion.z() / quaternion_norm);

  // Convert Pose3 to rerun::Transform3D
  auto transform =
      rerun::Transform3D()
          .with_translation({static_cast<float>(tf.x()),
                             static_cast<float>(tf.y()),
                             static_cast<float>(tf.z())})
          .with_quaternion(rerun::datatypes::Quaternion::from_wxyz(
              {q_w, q_x, q_y, q_z}))
          .with_relation(rerun::TransformRelation::ParentFromChild);

  // Rerun >=0.31 visualizes transform axes via a separate archetype.
  rec_->log_with_static(
      entity_path, is_static, transform, rerun::TransformAxes3D(axis_length));
}

}  // namespace aria::viz
