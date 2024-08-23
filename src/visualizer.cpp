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

}  // namespace aria::viz