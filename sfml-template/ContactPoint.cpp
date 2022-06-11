#include "ContactPoint.h"

bool ContactPoint::matches(const ContactPoint& other) const
{
	return (incPointIndex == other.incPointIndex
		&& refEdgeIndex == other.refEdgeIndex);
}

std::string ContactPoint::idAsString() const
{
	std::stringstream ss;
	ss << incPointIndex << ' ' << refEdgeIndex;
	return ss.str();
}
