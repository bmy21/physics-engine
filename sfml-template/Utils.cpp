#include "Utils.h"
#include "ContactPoint.h"

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

real zcross(const vec2& v, const vec2& w)
{
	// z component of v x w
	return v.x * w.y - v.y * w.x;
}

void drawLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, sf::Color col)
{
	sf::Vertex line[] = {
		{sf::Vector2f(p1), col},
		{sf::Vector2f(p2), col}
	};

	window.draw(line, 2, sf::Lines);
}

void drawThickLine(sf::RenderWindow& window, const vec2& p1, const vec2& p2, real width, sf::Color col)
{
	static sf::RectangleShape shape;

	shape.setOrigin(0, width / 2);
	shape.setPosition(p1);
	shape.setSize({ magnitude(p2 - p1), width });
	shape.setRotation(std::atan2(p2.y - p1.y, p2.x - p1.x) * 180 / pi);
	shape.setFillColor(col);

	window.draw(shape);
}


// Returns signed distance to plane defined by point ref and normal n
// eps is the plane half-thickness
std::pair<real, ClipRegion> getClipRegion(const vec2& n, const vec2& ref, real eps, const vec2& p)
{
	real d = dot(p - ref, n);

	if (d > eps)
	{
		return { d, ClipRegion::Out };
	}
	else if (d >= -eps)
	{
		return { d, ClipRegion::On };
	}
	else
	{
		return { d, ClipRegion::In };
	}
}


// Clips the line segment from cp1 to cp2 against the plane defined by point ref and normal n
// Adjusts the point coords and clip indices accordingly
// clipIndex is the index of point ref
// Returns false if both points are outside the plane; true otherwise
bool clip(const vec2& n, const vec2& ref, real eps, int clipPointIndex, ContactPoint& cp1, ContactPoint& cp2)
{
	using enum ClipRegion;

	auto [d1, R1] = getClipRegion(n, ref, eps, cp1.point);
	auto [d2, R2] = getClipRegion(n, ref, eps, cp2.point);

	if (R1 != Out && R2 != Out)
	{
		// Both points already on or in the plane, no clip required
		return true;
	}
	else if (R1 != Out && R2 == Out)
	{
		// Only point 2 is outside the plane
		cp2.clippedAgainstPoint = clipPointIndex;
		cp2.point = (d2 * cp1.point - d1 * cp2.point) / (d2 - d1);
		return true;
	}
	else if (R1 == Out && R2 != Out)
	{
		// Only point 1 is outside the plane
		cp1.clippedAgainstPoint = clipPointIndex;
		cp1.point = (d1 * cp2.point - d2 * cp1.point) / (d1 - d2);
		return true;
	}
	else 
	{
		// Both points outside the plane, no valid points can be returned
		return false;
	}
}