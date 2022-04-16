#pragma once
#include "Utils.h"


class ContactPoint
{
public:
	bool matches(const ContactPoint& other) const;
	std::string idAsString() const;

	vec2 point;
	real penetration = 0;

	int pointIndex = -1;
	int clippedAgainstEdge = -1;
	int clippedAgainstPoint = -1;

	

private:

};

