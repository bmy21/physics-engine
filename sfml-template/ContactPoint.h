#pragma once
#include "Utils.h"

enum class FeatureType
{
	Edge, 
	Point
};


class ContactPoint
{
public:
	vec2 point;
	real penetration = 0;

	//FeatureType typeA, typeB;
	int pointIndex = -1;
	int clippedAgainst = -1;

private:

};

