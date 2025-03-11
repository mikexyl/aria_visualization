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