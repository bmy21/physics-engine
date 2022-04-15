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

// Clip the line segment from point1 to point2 in direction dir using the edge defined by ref
std::vector<vec2> clip(const vec2& dir, const vec2& ref, const vec2& point1, const vec2& point2)
{
	// TODO: optimise for clip against both sides in one call?
	vec2 inner = point1;
	vec2 outer = point2;

	/*real projectedDifference = dot(point2 - point1, dir);

	if (projectedDifference < 0)
	{
		std::swap(inner, outer);
	}*/
	
	real projOuter = dot(outer - ref, dir);
	real projInner = dot(inner - ref, dir);

	if (projOuter < projInner)
	{
		std::swap(inner, outer);
		std::swap(projInner, projOuter);
	}


	if (projOuter <= 0)
	{
		// No clipping required, both lie inside the required region
		return { inner, outer };
	}

	else if (projInner > 0)
	{
		// Both points outside the required region, return no points
		return {};
	}

	else
	{
		// Return the inner point and the intersection point with the clip edge
		vec2 p = (projOuter * inner - projInner * outer) / (projOuter - projInner);
		return { inner, p };
	}

}
