#pragma once
#include "Utils.h"

struct AABB
{
	bool overlaps(const AABB& other) 
	{ 
		return rangeOverlaps({ lower.x, upper.x }, { other.lower.x, other.upper.x }) 
			&& rangeOverlaps({ lower.y, upper.y }, { other.lower.y, other.upper.y });
	}

	AABB unionWith(const AABB& other)
	{
		AABB u;

		u.lower.x = std::min(lower.x, other.lower.x);
		u.lower.y = std::min(lower.y, other.lower.y);
		u.upper.x = std::max(upper.x, other.upper.x);
		u.upper.y = std::max(upper.y, other.upper.y);

		return u;
	}

	// Perimeter - 2D analogue of SA
	real peri()
	{
		return 2 * (upper.x - lower.x + upper.y - lower.y);
	}

	vec2 lower, upper;
};