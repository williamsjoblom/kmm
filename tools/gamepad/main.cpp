#include <SFML/Window.hpp>
#include <iostream>

int main() {

  const int DEVICE_ID = 0;

  while (true) {
    sf::Joystick::update();

    float x = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::X);
    float y = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::Y);
    float u = sf::Joystick::getAxisPosition(DEVICE_ID, sf::Joystick::U);

    printf("\rx: %.2f, y: %.2f, u: %.2f", x, y, u);

    float R = 0.046;
    float L = 0.103; // [m]
    float sigma = 0;
    float a1 = (1.0 / R) * (-sin(sigma + r)*x + cos(sigma + r)*y + L*u)
    // float a2 = ...
    // float a3 = ...

  }

}
