#pragma once

#include "SFML/System.hpp"
#include "SFML/Graphics.hpp"
#include "SFML/Window.hpp"

#include <vector>
#include <memory>
#include <iostream>
#include <cmath>
#include <cassert>
#include <sstream>

class ContactPoint;


using real = float;
using vec2 = sf::Vector2<real>;

constexpr real pi = 3.14159265359;

vec2 rotate(const vec2& v, real theta);
void centre(sf::Text& text);

real dot(const vec2& v1, const vec2& v2);
vec2 perp(const vec2& v);
vec2 transform(const vec2& v, const vec2& offset, real angle);
real magnitude(const vec2& v);
vec2 normalise(const vec2& v); 
real zcross(const vec2& v, const vec2& w);
void drawLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, sf::Color col);
void drawThickLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, real width, sf::Color col);

enum class ClipRegion { In, On, Out };
std::pair<real, ClipRegion> getClipRegion(const vec2& n, const vec2& ref, real eps, const vec2& p);
bool clip(const vec2& n, const vec2& ref, real eps, int clipPointIndex, ContactPoint& cp1, ContactPoint& cp2);