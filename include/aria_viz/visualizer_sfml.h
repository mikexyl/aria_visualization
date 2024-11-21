#pragma once

#include <tbb/concurrent_queue.h>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/System/Sleep.hpp>
#include <SFML/Window/Event.hpp>
#include <rerun.hpp>
#include <stop_token>
#include <thread>

#include "aria_viz/visualizer.h"

namespace aria::viz {
class VisualizerSFML : public Visualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(VisualizerSFML);
  ARIA_POINTER_TYPEDEFS(VisualizerSFML);

  class Params : public Visualizer::Params {
   public:
    Params() {}

    float sfml_fps{60};
  };

  VisualizerSFML(Params params) : Visualizer(params), params_(params) {
    clear();
    render_thread_ = std::jthread(
        std::bind(&VisualizerSFML::renderTask, this, std::placeholders::_1));
  }

  virtual ~VisualizerSFML() {
    render_thread_.request_stop();
    render_thread_.join();
  }

  void drawPointsImpl(const std::string& entity_path,
                      const std::vector<Point3>& points,
                      const std::vector<Eigen::Vector4f>& rgba,
                      std::vector<float> radius,
                      bool is_static = false) override {
    for (size_t i = 0; i < points.size(); i++) {
      sf::CircleShape* circle = new sf::CircleShape(radius[i]);
      circle->setFillColor(
          sf::Color(rgba[i].x(), rgba[i].y(), rgba[i].z(), rgba[i].w()));
      circle->setPosition(points[i].x() - radius[i], points[i].y() - radius[i]);
      drawables_.push(circle);
    }
  }

  void clear() {
    // pop all shapes and delete
    sf::Drawable* shape;
    while (drawables_.try_pop(shape)) {
      delete shape;
    }
  }

  void renderTask(std::stop_token stop_token);

  bool windowOpened() const { return window_opened_; }

  bool frameReady() const { return frame_ready_; }

  void render() {
    frame_ready_ = false;
    render_frame_ = true;
  }

  void drawLines(const std::string& entity_path,
                 const std::vector<std::pair<Point3, Point3>>& points_pairs,
                 Eigen::Vector4f rgba,
                 float radius = 0.01f,
                 const std::vector<std::string>& labels = {}) override;

  void drawUncertaintyImpl2D(const std::string& entity_path,
                             const Point2& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width) override;

  void drawUncertaintyImpl3D(const std::string& entity_path,
                             const Point3& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width) override;

 private:
  std::jthread render_thread_;
  std::atomic<bool> window_opened_{false};
  std::atomic<bool> frame_ready_{true};
  std::atomic<bool> render_frame_{false};

  tbb::concurrent_queue<sf::Drawable*> drawables_;

  Params params_;
};
}  // namespace aria::viz