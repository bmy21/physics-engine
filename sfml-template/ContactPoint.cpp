#include "ContactPoint.h"

bool ContactPoint::matches(const ContactPoint& other) const
{
	// Note: not requiring clippedAgainstPoint to be matched
	return (incPointIndex == other.incPointIndex
		&& refEdgeIndex == other.refEdgeIndex);
}

std::string ContactPoint::idAsString() const
{
	std::stringstream ss;
	ss << incPointIndex << ' ' << refEdgeIndex << ' ' << clippedAgainstPoint;
	return ss.str();
}
