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
	int indexA = -1, indexB = -1;
	int clippedA = -1, clippedB = -1;

private:

};

