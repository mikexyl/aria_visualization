#include "aria_viz/visualizer_sfml.h"

#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/TGUI.hpp>

namespace aria::viz {

void VisualizerSFML::renderTask(std::stop_token stop_token) {
  float fps = params_.sfml_fps;
  const sf::Time target_frame_time = sf::seconds(1.0f / fps);

  sf::RenderWindow window(sf::VideoMode(params_.window_width_height.first,
                                        params_.window_width_height.second),
                          "VisualizerSFML",
                          sf::Style::Titlebar | sf::Style::Close);
  tgui::Gui gui(window);

  // Create a panel to act as a container
  auto panel = tgui::Panel::create();
  panel->setPosition(0, 0);  // Set position of the panel
  panel->setSize(100, 100);  // Set size of the panel
  panel->add(tgui_vertical_layout_);
  gui.add(panel);

  window.setVerticalSyncEnabled(false);

  while (!stop_token.stop_requested() and not window.isOpen());
  window.clear(sf::Color::White);
  window_opened_ = true;
  sf::Clock frame_clock;

  while (!stop_token.stop_requested()) {
    frame_ready_ = true;
    frame_clock.restart();

    sf::Event event;
    while (window.pollEvent(event)) {
      gui.handleEvent(event);

      if (event.type == sf::Event::Closed or
          // or press esc
          (event.type == sf::Event::KeyPressed and
           event.key.code == sf::Keyboard::Escape)) {
        window.close();
        window_opened_ = false;
        return;
      }
    }

    if (event.type == sf::Event::KeyPressed) {
      char key = event.key.code + 'A';
      std::cout << key << std::endl;
      if (keyboard_callbacks_.contains(key)) {
        keyboard_callbacks_.at(key)();
      }
      switch (event.key.code) {
        case sf::Keyboard::W:
          global_transform_ =
              global_transform_ * sf::Transform().translate(0, -10);
          break;
        case sf::Keyboard::S:
          global_transform_ =
              global_transform_ * sf::Transform().translate(0, 10);
          break;
        case sf::Keyboard::A:
          global_transform_ =
              global_transform_ * sf::Transform().translate(-10, 0);
          break;
        case sf::Keyboard::D:
          global_transform_ =
              global_transform_ * sf::Transform().translate(10, 0);
          break;
        default:
          break;
      }
    }

    if (render_frame_) {
      render_frame_ = false;
      window.clear(sf::Color::White);
      std::shared_ptr<sf::Drawable> shape;
      sf::RenderStates states(global_transform_);
      while (drawables_.try_pop(shape)) {
        window.draw(*shape, states);
      }

      for (auto& shape : static_drawables_) {
        window.draw(*shape, states);
      }

      tgui::Button::Ptr button;
      while (new_buttons_.try_pop(button)) {
        tgui_vertical_layout_->add(button);
      }

      gui.draw();
      window.display();
    }

    sf::Time elapsed_time = frame_clock.getElapsedTime();
    if (elapsed_time < target_frame_time) {
      frame_ready_ = false;
      sf::sleep(target_frame_time - elapsed_time);
    }
  }

  window.close();
  window_opened_ = false;
}

void VisualizerSFML::drawLinesImpl(
    const std::string& entity_path,
    const std::vector<std::pair<Point3, Point3>>& points_pairs,
    Eigen::Vector4f rgba,
    float radius,
    const std::vector<std::string>& labels,
    const std::vector<std::string>& text) {
  for (const auto& [p0, p1] : points_pairs) {
    auto line = std::make_shared<sf::VertexArray>(sf::LinesStrip, 2);
    (*line)[0].position = sf::Vector2f(p0.x(), p0.y());
    (*line)[1].position = sf::Vector2f(p1.x(), p1.y());
    (*line)[0].color = sf::Color(rgba[0], rgba[1], rgba[2], rgba[3]);
    (*line)[1].color = sf::Color(rgba[0], rgba[1], rgba[2], rgba[3]);
    drawables_.push(line);
  }
}

void VisualizerSFML::drawUncertaintyImpl2D(const std::string& entity_path,
                                           const Point2& mean,
                                           const std::vector<double>& ellipse,
                                           const Eigen::Vector4f& rgba,
                                           float radius,
                                           bool is_static) {
  float size_n_sigma = radius;
  double width = ellipse[0] * size_n_sigma, height = ellipse[1] * size_n_sigma,
         angle = ellipse[2];
  auto circle = std::make_shared<sf::CircleShape>(width);
  circle->setScale(1, height / width);
  circle->setFillColor(sf::Color(rgba[0], rgba[1], rgba[2], rgba[3]));
  circle->setRotation(angle);
  circle->setPosition(mean.x() - width, mean.y() - height);
  pushDrawable(circle, is_static);
}

void VisualizerSFML::drawUncertaintyImpl3D(const std::string& entity_path,
                                           const Point3& mean,
                                           const std::vector<double>& ellipse,
                                           const Eigen::Vector4f& rgba,
                                           float line_width,
                                           bool is_static) {
  Point2 mean2d(mean.x(), mean.y());
  drawUncertaintyImpl2D(entity_path,
                        mean2d,
                        {ellipse[0], ellipse[1], ellipse[5]},
                        rgba,
                        line_width,
                        is_static);
}

}  // namespace aria::viz