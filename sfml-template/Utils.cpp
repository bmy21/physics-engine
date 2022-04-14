#include "Utils.h"

vec2 rotate(const vec2& v, real theta)
{
    return { std::cos(theta) * v.x - std::sin(theta) * v.y,
             std::sin(theta) * v.x + std::cos(theta) * v.y };
}

void centre(sf::Text& text)
{
	auto globalBounds = text.getGlobalBounds();
	auto localBounds = text.getLocalBounds();

	text.setOrigin({ localBounds.left + globalBounds.width / 2, localBounds.top + globalBounds.height / 2 });
}

real dot(const vec2& v1, const vec2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

vec2 perp(const vec2& v)
{
	// v x (0, 0, 1)
	return { v.y, -v.x };
}

vec2 transform(const vec2& v, const vec2& offset, real angle)
{
	return offset + rotate(v, angle);
}

real magnitude(const vec2& v)
{
	return std::sqrt(v.x * v.x + v.y * v.y);
}

vec2 normalise(const vec2& v)
{
	return v / magnitude(v);
}

void drawLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, sf::Color col)
{
	sf::Vertex line[] = {
		{sf::Vector2f(p1), col},
		{sf::Vector2f(p2), col}
	};

	window.draw(line, 2, sf::Lines);
}