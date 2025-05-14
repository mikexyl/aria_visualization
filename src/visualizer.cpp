#include "aria_viz/visualizer.h"

namespace aria::viz {
std::map<int, Eigen::Vector3f> AgentColorMap::color_map = {
    {0, Eigen::Vector3f(255, 64, 64)},     // bright red
    {1, Eigen::Vector3f(64, 255, 64)},     // bright green
    {2, Eigen::Vector3f(128, 192, 255)},   // cyan-ish, bright for dark bg
    {3, Eigen::Vector3f(255, 255, 64)},    // yellow
    {4, Eigen::Vector3f(255, 64, 255)},    // magenta
    {5, Eigen::Vector3f(64, 255, 255)},    // cyan
    {6, Eigen::Vector3f(127, 255, 127)},   // mint green
    {7, Eigen::Vector3f(192, 64, 64)},     // soft red
    {8, Eigen::Vector3f(64, 192, 64)},     // soft green
    {9, Eigen::Vector3f(64, 64, 192)},     // soft blue
    {10, Eigen::Vector3f(192, 192, 64)},   // olive
    {11, Eigen::Vector3f(192, 64, 192)},   // violet
    {12, Eigen::Vector3f(64, 192, 192)},   // light teal
    {13, Eigen::Vector3f(192, 192, 192)},  // bright gray
    {14, Eigen::Vector3f(255, 128, 64)},   // orange
    {15, Eigen::Vector3f(128, 255, 64)},   // lime
    {16, Eigen::Vector3f(64, 128, 255)},   // light blue
    {17, Eigen::Vector3f(255, 200, 64)},   // gold
    {18, Eigen::Vector3f(255, 64, 128)},   // pink
    {19, Eigen::Vector3f(64, 255, 128)},   // aqua green
    {20, Eigen::Vector3f(180, 180, 255)}   // bluish white
};
const Eigen::Vector4f ColorMap::kGreen = Eigen::Vector4f(0, 255, 0, 255);
const Eigen::Vector4f ColorMap::kRed = Eigen::Vector4f(255, 0, 0, 255);
const Eigen::Vector4f ColorMap::kBlue = Eigen::Vector4f(0, 0, 255, 255);
const Eigen::Vector4f ColorMap::kGray = Eigen::Vector4f(128, 128, 128, 255);
const Eigen::Vector4f ColorMap::kBlack = Eigen::Vector4f(0, 0, 0, 255);

class Visualizer;

void Visualizer::drawPoints(const std::string& entity_path,
                            const Values& values,
                            const std::vector<Eigen::Vector4f>& rgba,
                            std::vector<float> radius,
                            std::vector<std::string> labels,
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

  drawPointsImpl(
      entity_path, points, rgba_full, radius_full, labels, is_static);
}

std::vector<double> Visualizer::getEllipseFromCov(const Eigen::Matrix2d& cov) {
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

std::vector<double> Visualizer::getEllipseFromCov(const Eigen::Matrix3d& cov) {
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

template <>
void Visualizer::drawFactors(const std::string& entity_path,
                             const FactorGraph<NonlinearFactor>& factors,
                             const Values& values,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static);

template <>
void Visualizer::drawFactors(const std::string& entity_path,
                             const FactorGraph<GaussianFactor>& factors,
                             const Values& values,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static);

}  // namespace aria::viz