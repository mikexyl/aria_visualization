#include <aria_viz/visualizer_sfml.h>

using namespace aria::viz;

int main() {
  aria::viz::VisualizerSFML::Params params;
  aria::viz::VisualizerSFML viz(params);

  while (not viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  int it = 0;
  while (true) {
    it++;
    std::vector<Point3> samples;
    std::vector<float> radius;
    samples.push_back(Point3((200 + it * 10) % 800, 200, 0));
    samples.push_back(Point3((400 + it * 10) % 800, 200, 0));
    samples.push_back(Point3((600 + it * 10) % 800, 200, 0));
    radius.push_back(10);
    radius.push_back(10);
    radius.push_back(10);

    std::vector<Eigen::Vector4f> rgba;
    rgba.resize(samples.size(), Eigen::Vector4f(255, 0, 0, 255));

    // viz.clear();
    viz.drawPointsImpl("samples", samples, rgba, radius, true);

    viz.render();
    // while(not viz.frameReady()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    // }
  }

  // wait for window close
  std::cout << "Waiting for window close" << std::endl;
  while (viz.windowOpened()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}