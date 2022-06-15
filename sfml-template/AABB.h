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
		//return
		//{
		//	std::min(left, other.left),
		//	std::max(top, other.top),
		//	std::
		//};
	}

	vec2 lower, upper;
};