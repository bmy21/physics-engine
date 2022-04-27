#pragma once
#include "Utils.h"


class ContactPoint
{
public:
	bool matches(const ContactPoint& other) const;
	std::string idAsString() const;

	vec2 point;
	real penetration = 0;

	// Complete set of information that specifies how this 
	// contact point was generated. Can be used to recompute
	// the contact point or to match agatinst other contact points.

	// Index of point on incident body
	int incPointIndex = -1;

	// Index of reference edge
	int refEdgeIndex = -1;

	// Index of reference point defining side-plane against which 
	// the incident point was clipped. -1 for no clip.
	int clippedAgainstPoint = -1;

private:

};

