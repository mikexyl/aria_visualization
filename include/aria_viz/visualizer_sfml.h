#pragma once

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/System/Sleep.hpp>
#include <SFML/Window/Event.hpp>
#include <condition_variable>
#include <queue>
#include <rerun.hpp>
#include <stop_token>
#include <thread>

#include "aria_viz/visualizer.h"
#include "aria_viz/visualizer_rerun.h"

namespace aria::viz {
class VisualizerSFML : public Visualizer {
 public:
  ARIA_DELETE_COPY_CONSTRUCTORS(VisualizerSFML);
  ARIA_POINTER_TYPEDEFS(VisualizerSFML);

  class Params : public Visualizer::Params {
   public:
    Params() {}

    bool wait_for_first_image{true};
    float sfml_fps{30};
  };

  VisualizerSFML(Params params) : Visualizer(params), params_(params) {
    render_texture_.create(800, 600);
    clear();
    render_thread_ = std::jthread(
        std::bind(&VisualizerSFML::renderTask, this, std::placeholders::_1));
  }

  virtual ~VisualizerSFML() {
    render_thread_.request_stop();
    render_thread_.join();
  }

  void visualizePoints(const std::string& entity_path,
                       const std::vector<Point3>& points,
                       const std::vector<Eigen::Vector4f>& rgba,
                       std::vector<float> radius,
                       bool is_static = false) override {
    std::lock_guard<std::mutex> lock(render_mutex_);
    for (size_t i = 0; i < points.size(); i++) {
      sf::CircleShape circle(radius[i]);
      circle.setFillColor(
          sf::Color(rgba[i][0], rgba[i][1], rgba[i][2], rgba[i][3]));
      circle.setPosition(points[i].x(), points[i].y());
      render_texture_.draw(circle);
    }
  }

  void clear() {
    std::lock_guard<std::mutex> lock(render_mutex_);
    render_texture_.clear(sf::Color::White);
  }

  void renderTask(std::stop_token stop_token) {
    float fps = params_.sfml_fps;
    const sf::Time target_frame_time = sf::seconds(1.0f / fps);

    sf::RenderWindow window(sf::VideoMode(800, 600),
                            "VisualizerSFML",
                            sf::Style::Titlebar | sf::Style::Close);
    window.setVerticalSyncEnabled(false);

    while (!stop_token.stop_requested() and not window.isOpen());
    window.clear(sf::Color::White);
    window_opened_ = true;
    sf::Clock frame_clock;

    // create sprite of the render texture
    sf::Sprite sprite(render_texture_.getTexture());

    while (!stop_token.stop_requested()) {
      frame_clock.restart();

      sf::Event event;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
          window_opened_ = false;
          return;
        }
      }

      window.clear(sf::Color::White);
      {
        std::lock_guard<std::mutex> lock(render_mutex_);
        window.draw(sprite);
      }

      window.display();

      sf::Time elapsed_time = frame_clock.getElapsedTime();
      if (elapsed_time < target_frame_time) {
        sf::sleep(target_frame_time - elapsed_time);
      }
    }

    window.close();
    window_opened_ = false;
  }

  bool windowOpened() const { return window_opened_; }

 private:
  std::jthread render_thread_;
  std::atomic<bool> window_opened_{false};

  mutable std::mutex render_mutex_;
  sf::RenderTexture render_texture_;

  Params params_;
};
}  // namespace aria::viz