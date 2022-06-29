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
#include <array>
#include <utility>
#include <tuple>
#include <set>
#include <unordered_set>
#include <execution>
#include <forward_list>

struct ContactPoint;

using real = float;
using vec2 = sf::Vector2<real>;
constexpr real pi = 3.14159265359;
enum class Voronoi { Vertex, Edge, Inside };

void centre(sf::Text& text);

real dot(const vec2& v1, const vec2& v2);
vec2 perp(const vec2& v);
bool isZero(const vec2& v);

vec2 rotate(const vec2& v, real theta);
vec2 transform(const vec2& v, const vec2& offset, real angle);
vec2 invTransform(const vec2& v, const vec2& offset, real angle);

real magSquared(const vec2& v);
real magnitude(const vec2& v);
vec2 normalise(const vec2& v); 
real zcross(const vec2& v, const vec2& w);

std::pair<real, real> bary(const vec2& q, const vec2& v1, const vec2& v2);
std::tuple<real, real, real> bary(const vec2& q, const vec2& v1, const vec2& v2, const vec2& v3);
std::tuple<real, real, real> nonNormalisedBary(const vec2& q, const vec2& v1, const vec2& v2);
std::tuple<real, real, real, real> nonNormalisedBary(const vec2& q, const vec2& v1, const vec2& v2, const vec2& v3);

bool rangeOverlaps(const std::pair<real, real>& R1, const std::pair<real, real>& R2);
bool rangeContains(const std::pair<real, real>& R1, const std::pair<real, real>& R2);

real decayConstant(real halfLife);

void drawLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, sf::Color col);
void drawThickLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, real width, sf::Color col);

enum class ClipRegion { In, On, Out };
std::pair<real, ClipRegion> getClipRegion(const vec2& n, const vec2& ref, real eps, const vec2& p);
bool clip(const vec2& n, const vec2& ref, real eps, ContactPoint& cp1, ContactPoint& cp2);