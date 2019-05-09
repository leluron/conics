#pragma once

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <glm/glm.hpp>
#include <vector>

#include "trajectory.hpp"

class Display {

  sf::RenderWindow window;
  float aspect;
  glm::vec2 windowSize;
  glm::dvec3 center;
  double scale;

  struct planet {
    glm::dvec3 center;
    double radius;
  };

  std::vector<planet> planets;

  struct trajectory {
    glm::dvec3 center;
    OrbitalElements oe;
    double epochStart;
    double epochEnd;
    int segments;
  };

  std::vector<trajectory> trajectories;

public:
  void createWindow() {
    window.create(sf::VideoMode::getFullscreenModes()[0], "conics");
    window.setFramerateLimit(60);
    windowSize = glm::vec2(window.getSize().x, window.getSize().y);
    aspect = windowSize.x/windowSize.y;

    center = glm::dvec3(0,0,0);

    scale = 100.0;

    window.setActive();
  }

  bool isOpen() {
    return window.isOpen();
  }

  void clear() {
    planets.clear();
    trajectories.clear();
  }

  void addPlanet(glm::dvec3 center, double radius) {
    planets.push_back({center, radius});
  }

  void addTrajectory(glm::dvec3 center, OrbitalElements oe, double epochStart, double epochEnd, int segments) {
    trajectories.push_back({center, oe, epochStart, epochEnd, segments});
  }

  void setCenter(glm::dvec3 center) {
    this->center = center;
  }

  void setScale(double scale) {
    this->scale = scale;
  }

  glm::vec2 worldToView(glm::dvec3 pos) {
    return glm::vec2(((pos.x-center.x)*windowSize.x/scale)+windowSize.x*0.5, ((pos.y-center.y)*windowSize.x/scale)+windowSize.y*0.5);
  }

  void draw() {
    window.clear(sf::Color::Black);

    for (auto p : planets) {
      float size = (p.radius*windowSize.x)/scale;
      sf::CircleShape shape(size);
      shape.setFillColor(sf::Color::Cyan);
      auto relPos = p.center-center;
      shape.setPosition(relPos.x-size+windowSize.x*0.5, relPos.y*aspect-size+windowSize.y*0.5);
      window.draw(shape);
    }

    for (auto t : trajectories) {
      std::vector<sf::Vertex> verts(t.segments);
      double duration = t.epochEnd-t.epochStart;
      double deltaT = duration/(t.segments-1);
      for (int i=0;i<t.segments;i++) {
        glm::dvec3 pos = t.center + toStateVector(t.oe, t.epochStart+deltaT*i).r();
        auto view = worldToView(pos);
        verts[i] = sf::Vertex(sf::Vector2f(view.x, view.y));
      }
      window.draw(verts.data(), verts.size(), sf::LineStrip);
    }

    window.display();
  }

  void manageEvents() {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) window.close();
    }
  }
};

