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

// Clip the line segment from point1 to point2 in direction dir using the edge defined by ref
std::vector<vec2> clip(const vec2& dir, const vec2& ref, const vec2& point1, const vec2& point2, ClipType& type)
{
	//TODO: don't return a vector of vec2s, just the clipped point & outcome?
	vec2 inner = point1;
	vec2 outer = point2;

	real projOuter = dot(outer - ref, dir);
	real projInner = dot(inner - ref, dir);

	bool swapped = false;

	if (projOuter < projInner)
	{
		std::swap(inner, outer);
		std::swap(projInner, projOuter);
		swapped = true;
	}

	//std::cout << projInner << " " << projOuter << '\n';

	real tol = 0; // 1e-3;

	if (projOuter <= -tol)
	{
		// No clipping required, both lie inside the required region
		type = ClipType::None;
		return { point1, point2 };
	}
	else if (projInner > tol)
	{
		// Both points outside the required region, return no points
		type = ClipType::Both;
		return {};
	}
	else
	{
		// Return the inner point and the intersection point with the clip edge
		vec2 p = (projOuter * inner - projInner * outer) / (projOuter - projInner);

		if (swapped)
		{
			// point1 was clipped
			type = ClipType::First;
			return  { p, point2 };
		}
		else
		{
			// point2 was clipped
			type = ClipType::Second;
			return { point1, p };
		}
	}
}

// Returns signed distance to plane defined by point ref and normal n.
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


// Clips the line segment from cp1 to cp2 against the plane defined by point ref and normal n.
// Adjusts the point coords and clip indices accordingly.
// clipIndex is the index of point ref.
// Return values indicate whether the points are valid (true) or should be discarded (false).
std::pair<bool, bool>
clip(const vec2& n, const vec2& ref, real eps,
	int clipPointIndex, ContactPoint& cp1, ContactPoint& cp2)
{
	auto [d1, R1] = getClipRegion(n, ref, eps, cp1.point);
	auto [d2, R2] = getClipRegion(n, ref, eps, cp2.point);

	if (R1 == ClipRegion::On && R2 == ClipRegion::On
		|| R1 == ClipRegion::In && R2 == ClipRegion::In
		|| R1 == ClipRegion::On && R2 == ClipRegion::In
		|| R1 == ClipRegion::In && R2 == ClipRegion::On)
	{
		// no clip required
		return { true, true };
	}
	else if (R1 == ClipRegion::Out && R2 == ClipRegion::Out)
	{
		// no valid points
		return { false, false };
	}
	else if (R1 == ClipRegion::On && R2 == ClipRegion::Out)
	{
		// only p1 is valid
		return { true, false };
	}
	else if (R1 == ClipRegion::Out && R2 == ClipRegion::On)
	{
		// only p2 is valid
		return { false, true };
	}
	else if (R1 == ClipRegion::In && R2 == ClipRegion::Out)
	{
		// keep p1 and clipped p2
		cp2.clippedAgainstPoint = clipPointIndex;
		cp2.point = (d2 * cp1.point - d1 * cp2.point) / (d2 - d1);
		return { true, true };
	}
	else
	{
		// keep p2 and clipped p1
		cp1.clippedAgainstPoint = clipPointIndex;
		cp1.point = (d1 * cp2.point - d1 * cp1.point) / (d1 - d2);
		return { true, true };
	}
}