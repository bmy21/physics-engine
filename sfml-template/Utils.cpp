#include "Utils.h"
#include "ContactPoint.h"

vec2 rotate(const vec2& v, real theta)
{
	real c = std::cos(theta), s = std::sin(theta);

    return { c * v.x - s * v.y,
             s * v.x + c * v.y };
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

bool isZero(const vec2& v)
{
	return v.x == 0 && v.y == 0;
}

vec2 transform(const vec2& v, const vec2& offset, real angle)
{
	return offset + rotate(v, angle);
}

vec2 invTransform(const vec2& v, const vec2& offset, real angle)
{
	return rotate(v - offset, -angle);
}

real magSquared(const vec2& v)
{
	return v.x * v.x + v.y * v.y;
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

std::pair<real, real> bary(const vec2& q, const vec2& v1, const vec2& v2)
{
	vec2 v12 = v2 - v1;
	real msq = magSquared(v12);

	real u = dot(v2 - q, v12) / msq;
	real v = dot(q - v1, v12) / msq;

	return { u, v };
}

std::tuple<real, real, real> bary(const vec2& q, const vec2& v1, const vec2& v2, const vec2& v3)
{
	real a123 = zcross(v2 - v1, v3 - v1);
	real aq23 = zcross(v2 - q, v3 - q);
	real a1q3 = zcross(q - v1, v3 - v1);
	real a12q = zcross(v2 - v1, q - v1);

	real u = aq23 / a123;
	real v = a1q3 / a123;
	real w = a12q / a123;

	return { u, v, w };
}

std::tuple<real, real, real> nonNormalisedBary(const vec2& q, const vec2& v1, const vec2& v2)
{
	vec2 v12 = v2 - v1;
	real msq = magSquared(v12);

	real u = dot(v2 - q, v12);
	real v = dot(q - v1, v12);

	return { u, v, msq };
}

std::tuple<real, real, real, real> nonNormalisedBary(const vec2& q, const vec2& v1, const vec2& v2, const vec2& v3)
{
	real a123 = zcross(v2 - v1, v3 - v1);

	real u = zcross(v2 - q, v3 - q); // aq23
	real v = zcross(q - v1, v3 - v1); // a1q3
	real w = zcross(v2 - v1, q - v1); // a12q

	return { u, v, w, a123 };
}

bool rangeOverlaps(const std::pair<real, real>& R1, const std::pair<real, real>& R2)
{
	return (R1.first <= R2.second) && (R1.second >= R2.first);
}

bool rangeContains(const std::pair<real, real>& R1, const std::pair<real, real>& R2)
{
	// True if R1 contains R2
	return (R1.first <= R2.first) && (R1.second >= R2.second);
}

real decayConstant(real halfLife)
{
	return std::log(2) / halfLife;
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
	shape.setPosition(p1.x, p1.y);

	vec2 size = { magnitude(p2 - p1), width };
	shape.setSize(sf::Vector2f(size.x, size.y));

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
// Returns "success" - false if both points are outside the plane; true otherwise
bool clip(const vec2& n, const vec2& ref, real eps, ContactPoint& cp1, ContactPoint& cp2)
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
		cp2.point = (d2 * cp1.point - d1 * cp2.point) / (d2 - d1);
		return true;
	}
	else if (R1 == Out && R2 != Out)
	{
		// Only point 1 is outside the plane
		cp1.point = (d1 * cp2.point - d2 * cp1.point) / (d1 - d2);
		return true;
	}
	else 
	{
		// Both points outside the plane, no valid points can be returned
		return false;
	}
}