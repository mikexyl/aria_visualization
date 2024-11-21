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
    VisualizerRerun::Pose3RendererSFML pose3_renderer_;
    auto sf_image = pose3_renderer_.pointsToSfImage(points, rgba, radius);
    std::lock_guard<std::mutex> lock(images_mutex_);
    images_.push(sf_image);
    std::cout << "image queue size: " << std::endl;
  }

  void renderTask(std::stop_token stop_token) {
    float fps = params_.sfml_fps;
    const sf::Time target_frame_time = sf::seconds(1.0f / fps);

    // wait until there is an image to render
    while (!stop_token.stop_requested() and images_.empty() and
           params_.wait_for_first_image) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(target_frame_time.asMilliseconds()) / 2);
    }

    sf::RenderWindow window(sf::VideoMode(800, 600),
                            "VisualizerSFML",
                            sf::Style::Titlebar | sf::Style::Close);
    window.setVerticalSyncEnabled(false);

    while (!stop_token.stop_requested() and not window.isOpen());
    window.clear(sf::Color::White);
    window_opened_ = true;
    sf::Clock frame_clock;

    while (!stop_token.stop_requested()) {
      std::cout << "draw image" << std::endl;
      frame_clock.restart();

      std::cout << "1" << std::endl;
      window.clear(sf::Color::White);
      std::cout << "2" << std::endl;

      sf::Event event;
      std::cout << "polling" << std::endl;
      while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
          window.close();
          window_opened_ = false;
          return;
        }
      }

      std::cout << "trying to pop" << std::endl;
      {
        std::lock_guard<std::mutex> lock(images_mutex_);
        if (!images_.empty()) {
          auto image = images_.front();
          images_.pop();
          sf::Texture texture;
          texture.loadFromImage(image);
          sf::Sprite sprite(texture);
          window.draw(sprite);
        }
      }

      std::cout << "trying to display" << std::endl;
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

  bool frameFinished() const {
    std::lock_guard<std::mutex> lock(images_mutex_);
    return images_.empty();
  }

 private:
  std::jthread render_thread_;
  std::atomic<bool> window_opened_{false};

  mutable std::mutex images_mutex_;
  std::queue<sf::Image> images_;

  Params params_;
};
}  // namespace aria::viz