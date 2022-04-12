#pragma once

#include "SFML/System.hpp"
#include "SFML/Graphics.hpp"
#include "SFML/Window.hpp"

#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <cassert>

using real = float;
using vec2 = sf::Vector2<real>;

constexpr real pi = 3.14159265359;

vec2 rotate(const vec2& v, real theta);
void centre(sf::Text& text);