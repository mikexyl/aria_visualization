#pragma once

#include <tbb/concurrent_map.h>
#include <tbb/concurrent_queue.h>

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderTexture.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/Graphics/Sprite.hpp>
#include <SFML/System/Sleep.hpp>
#include <SFML/Window/Event.hpp>
#include <TGUI/TGUI.hpp>
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
    std::pair<int, int> window_width_height{800, 600};
    std::pair<int, int> offset{0, 0};
    std::pair<float, float> scale{1.0, 1.0};
  };

  VisualizerSFML(Params params = {}) : Visualizer(params), params_(params) {
    clear();
    global_transform_ = sf::Transform::Identity;
    global_transform_.translate(params_.offset.first, params_.offset.second);
    global_transform_.scale(params_.scale.first, params_.scale.second);
    tgui_vertical_layout_ = tgui::VerticalLayout::create();
    tgui_vertical_layout_->setPosition(0, 0);
    tgui_vertical_layout_->setAutoLayout(tgui::AutoLayout::Top);
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
                      const std::vector<float>& radius,
                      const std::vector<std::string>& labels,
                      bool is_static = false) override {
    for (size_t i = 0; i < points.size(); i++) {
      auto circle = std::make_shared<sf::CircleShape>(radius[i]);
      circle->setFillColor(
          sf::Color(rgba[i].x(), rgba[i].y(), rgba[i].z(), rgba[i].w()));
      circle->setPosition(points[i].x() - radius[i], points[i].y() - radius[i]);
      if (is_static) {
        static_drawables_.push_back(circle);
      } else {
        drawables_.push(circle);
      }
    }
  }

  void clear(bool clear_static = false) {
    // pop all shapes and delete
    drawables_.clear();

    if (clear_static) {
      static_drawables_.clear();
    }
  }

  void renderTask(std::stop_token stop_token);

  bool windowOpened() const { return window_opened_; }

  bool frameReady() const { return frame_ready_; }

  bool waitForWindowClose(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) {
    auto start = std::chrono::system_clock::now();
    while (window_opened_) {
      if (timeout.count() > 0) {
        auto now = std::chrono::system_clock::now();
        if (now - start > timeout) {
          return false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
  }

  bool waitForWindowOpen(
      std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) {
    auto start = std::chrono::system_clock::now();
    while (not window_opened_) {
      if (timeout.count() > 0) {
        auto now = std::chrono::system_clock::now();
        if (now - start > timeout) {
          return false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
  }

  void render() {
    frame_ready_ = false;
    render_frame_ = true;
  }

  void addButton(std::string text, std::function<void()> callback) {
    tgui::Button::Ptr button = tgui::Button::create(text);
    button->onPress(callback);
    new_buttons_.push(button);
  }

  void drawLinesImpl(const std::string& entity_path,
                     const std::vector<std::pair<Point3, Point3>>& points_pairs,
                     const std::vector<Eigen::Vector4f>& rgba,
                     float radius,
                     const std::vector<std::string>& labels,
                     const std::vector<std::string>& text) override;

  void drawUncertaintyImpl2D(const std::string& entity_path,
                             const Point2& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static) override;

  void drawUncertaintyImpl3D(const std::string& entity_path,
                             const Point3& mean,
                             const std::vector<double>& ellipse,
                             const Eigen::Vector4f& rgba,
                             float line_width,
                             bool is_static) override;

  void addKeyboardCallback(char key, std::function<void()> callback) {
    keyboard_callbacks_.insert({key, callback});
  }

 private:
  void pushDrawable(std::shared_ptr<sf::Drawable> drawable, bool is_static) {
    if (is_static) {
      static_drawables_.push_back(drawable);
    } else {
      drawables_.push(drawable);
    }
  }

 private:
  std::jthread render_thread_;
  std::atomic<bool> window_opened_{false};
  std::atomic<bool> frame_ready_{true};
  std::atomic<bool> render_frame_{false};

  tbb::concurrent_queue<std::shared_ptr<sf::Drawable>> drawables_;
  tbb::concurrent_vector<std::shared_ptr<sf::Drawable>> static_drawables_;
  sf::Transform global_transform_;

  tbb::concurrent_queue<tgui::Button::Ptr> new_buttons_;
  tbb::concurrent_map<char, std::function<void()>> keyboard_callbacks_;

  tgui::VerticalLayout::Ptr tgui_vertical_layout_;

  Params params_;
};
}  // namespace aria::viz