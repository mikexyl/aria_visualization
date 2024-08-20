#include "aria_visualization/frame_visualizer_rerun.h"

#include <aria_common/logging.h>

#include "collection_adapters.hpp"

using namespace gtsam;

namespace aria::visualization {

static rerun::Collection<rerun::TensorDimension> tensor_shape(
    const cv::Mat& img) {
  return {img.rows, img.cols, img.channels()};
};

void FrameVisualizerRerun::setTimeNSec(size_t timestamp) {
  rec_->set_time_nanos("time", timestamp);
}

void FrameVisualizerRerun::visualize(const Frame::Ptr& frame) {
  // update timeline
  // auto now = std::chrono::system_clock::now();
  // auto now_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
  //                      now.time_since_epoch())
  //                      .count();
  // rec_->set_time_nanos("time", now_in_ns);
  agent_id_ = frame->getAgentId();

  setTimeNSec(frame->getTimestamp());

  visualizeCameraPose(frame);

  FrameVisualizer::visualize(frame);
}

void FrameVisualizerRerun::visualizeTrackingImage(const cv::Mat& image,
                                                  const Frame::Ptr& frame) {
  // cv::Mat rgb_image;
  // cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);

  // // log the image
  // rec_->log("odom/frame" + std::to_string(frame->getFrameId()) + "/image",
  //           rerun::Image(tensor_shape(rgb_image),
  //                        rerun::TensorBuffer::u8(rgb_image)));

  FrameVisualizer::visualizeTrackingImage(image, frame);
}

void FrameVisualizerRerun::visualizeLandmarks(
    const std::map<LandmarkId, Point3>& landmarks,
    std::string frame_id,
    size_t timestamp) {}

void FrameVisualizerRerun::visualizeLandmarks(
    const Frame::Ptr& frame,
    const FrameDatabase::Ptr& frame_db,
    bool use_all_frames) {
  if (!agent_id_) {
    return;
  }
  std::vector<Eigen::Vector3f> landmarks;
  for (const auto& [id, landmark] :
       use_all_frames ? frame_db->getLandmarks() : frame->getLandmarks()) {
    landmarks.push_back(landmark->cast<float>());
  }

  spdlog::info("visualize agent id: {}, landmarks: {}",
               frame->getAgentId(),
               landmarks.size());

  std::string entity_path = "odom/" + std::to_string(*agent_id_) + "/" +
                            std::to_string(frame->getAgentId()) + "/landmarks";

  rerun::Color landmark_color = fromEigen(
      AgentColorMap::color_map[frame->getAgentId()], kLandmarkColorAlpha);
  rec_->log_static(entity_path,
                   rerun::Points3D(landmarks).with_radii(0.01f).with_colors(
                       {landmark_color}));
}

void FrameVisualizerRerun::visualizeExtFrame(const ExtFrame::Ptr& ext_frame,
                                             size_t timestamp) {
  if (!agent_id_) {
    return;
  }
  rec_->set_time_nanos("time", timestamp);

  // if no ext frame, this function can be used to update rerun's timestamp
  if (not ext_frame) return;

  Eigen::Vector3f camera_pos = ext_frame->getPose().translation().cast<float>();
  Eigen::Matrix3f camera_orientation =
      ext_frame->getPose().rotation().matrix().cast<float>();
  std::vector<Eigen::Vector3f> camera_points{camera_pos};
  rerun::Color ext_frame_pose_color =
      fromEigen(AgentColorMap::color_map[ext_frame->getAgentId()]);
  rec_->log("odom/" + std::to_string(*agent_id_) + "/" +
                std::to_string(ext_frame->getAgentId()) + "/frame/" +
                std::to_string(ext_frame->getFrameId()),
            rerun::Points3D(camera_points)
                .with_colors({ext_frame_pose_color})
                .with_radii(0.02f));

  // visualize landmarks
  std::vector<Eigen::Vector3f> landmarks(ext_frame->getLandmarks().size());
  size_t i = 0;
  for (const auto& [id, landmark] : ext_frame->getLandmarks()) {
    // landmark is T_we_landmark
    auto T_ext_landmark = ext_frame->getExtPose().value().inverse() *
                          *landmark;  // T_ext_we * T_we_landmark
    auto T_wl_landmark =
        ext_frame->getPose() * T_ext_landmark;  // T_wl_ext * T_ext_landmark
    landmarks[i++] = T_wl_landmark.cast<float>();
  }

  std::string entity_path =
      "odom/" + std::to_string(*agent_id_) + "/" +
      std::to_string(ext_frame->getAgentId()) + "/frame/" +
      std::to_string(ext_frame->getFrameId()) + "/landmarks";

  rerun::Color landmark_color = fromEigen(
      AgentColorMap::color_map[ext_frame->getAgentId()], kLandmarkColorAlpha);
  rec_->log_static(entity_path,
                   rerun::Points3D(landmarks).with_radii(0.01f).with_colors(
                       {landmark_color}));
}

void FrameVisualizerRerun::visualizeCameraPose(const Frame::Ptr& frame) {
  Eigen::Vector3f camera_pos =
      frame->getCamera()->pose().translation().cast<float>();
  Eigen::Matrix3f camera_orientation =
      frame->getCamera()->pose().rotation().matrix().cast<float>();
  std::vector<Eigen::Vector3f> camera_points{camera_pos};
  rerun::Color camera_color =
      fromEigen(AgentColorMap::color_map[frame->getAgentId()]);
  rec_->log("odom/" + std::to_string(*agent_id_) + "/" +
                std::to_string(frame->getAgentId()) + "/frame/" +
                std::to_string(frame->getFrameId()),
            rerun::Points3D(camera_points)
                .with_colors({camera_color})
                .with_radii(0.02f));
}

void FrameVisualizerRerun::connectFrames(
    const std::string& entity_path,
    Eigen::Vector4f rgba,
    std::vector<std::pair<Frame::Ptr, Frame::Ptr>>& frame_pairs) {
  if (frame_pairs.size() == 0) {
    return;
  }

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [frame, ext_frame] : frame_pairs) {
    positions.push_back({frame->getPose().translation().cast<float>(),
                         ext_frame->getPose().translation().cast<float>()});
  }

  connectPositions3D(entity_path, positions, rgba);
}

void FrameVisualizerRerun::connectPositions3D(
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

void FrameVisualizerRerun::connectLandmarks(
    const std::string& entity_path,
    Eigen::Vector4f rgba,
    std::vector<LandmarkPair>& landmark_pairs) {
  if (landmark_pairs.size() == 0) {
    return;
  }
  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [landmark_0, landmark_1] : landmark_pairs) {
    positions.push_back({landmark_0->cast<float>(), landmark_1->cast<float>()});
  }

  connectPositions3D(entity_path, positions, rgba);
}

void FrameVisualizerRerun::connectFramesToLandmarks(
    const std::string& entity_path,
    Eigen::Vector4f rgba,
    std::map<Landmark::Ptr, FrameSet>& landmarks_and_frames) {
  if (landmarks_and_frames.size() == 0) {
    return;
  }

  std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> positions;
  for (auto const& [landmark, frames] : landmarks_and_frames) {
    for (auto const& frame : frames) {
      positions.push_back({frame->getPose().translation().cast<float>(),
                           landmark->cast<float>()});
    }
  }

  connectPositions3D(entity_path, positions, rgba);
}

void FrameVisualizerRerun::connectPointsToPoints(
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

float FrameVisualizerRerun::visualizePoints(
    const std::string& entity_path,
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

void FrameVisualizerRerun::visualizePoints(
    const std::string& entity_path,
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
void FrameVisualizerRerun::visualizeUncertainty(const std::string& entity_path,
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

void FrameVisualizerRerun::visualizeFactors(const std::string& entity_path,
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

std::vector<Point3> FrameVisualizerRerun::generateEllipse(
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

}  // namespace aria::visualization