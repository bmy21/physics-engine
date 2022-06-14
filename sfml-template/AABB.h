#pragma once
#include "Utils.h"

struct AABB
{
	bool overlaps(const AABB& other) 
	{ 
		return rangeOverlaps({ left, right }, { other.left, other.right }) 
			&& rangeOverlaps({ top, bottom}, { other.top, other.bottom });
	}

	real left, bottom, right, top;
};