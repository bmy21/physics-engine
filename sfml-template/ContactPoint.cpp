#include "ContactPoint.h"

bool ContactPoint::matches(const ContactPoint& other) const
{
	return (pointIndex == other.pointIndex 
		&& clippedAgainstEdge == other.clippedAgainstEdge
		&& clippedAgainstPoint == other.clippedAgainstPoint);
}

std::string ContactPoint::idAsString() const
{
	std::stringstream ss;
	ss << pointIndex << ' ' << clippedAgainstEdge << ' ' << clippedAgainstPoint;
	return ss.str();
}
