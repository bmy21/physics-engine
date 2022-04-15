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
std::vector<vec2> clip(const vec2& dir, const vec2& ref, const vec2& point1, const vec2& point2, int& nclips)
{
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

	if (projOuter <= 0)
	{
		// No clipping required, both lie inside the required region
		nclips = 0;
		return { point1, point2 };
	}
	else if (projInner > 0)
	{
		// Both points outside the required region, return no points
		
		nclips = 2;
		return {};
	}
	else
	{
		// Return the inner point and the intersection point with the clip edge
		nclips = 1;
		vec2 p = (projOuter * inner - projInner * outer) / (projOuter - projInner);

		if (swapped)
		{
			// point1 was clipped
			return  { p, point2 };
		}
		else
		{
			// point2 was clipped
			return { point1, p };
		}
	}
}

std::vector<vec2> clip(const vec2& dir, const vec2& ref, const vec2& point1, const vec2& point2, ClipType& type)
{
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

	if (projOuter <= 0)
	{
		// No clipping required, both lie inside the required region
		type = ClipType::None;
		return { point1, point2 };
	}
	else if (projInner > 0)
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
