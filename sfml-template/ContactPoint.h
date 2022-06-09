#pragma once
#include "Utils.h"

class ContactPoint
{
public:
	bool matches(const ContactPoint& other) const;
	std::string idAsString() const;

	vec2 point;
	real penetration = 0;

	real lambda = 0;
	real fLambda = 0;

	real vRelTarget = 0;

	// A point on the incident body used to generate the contact point
	vec2 localIncPoint;

	// z.(r x n) and z.(r x t) for bodies 1 & 2
	real nCrossFactor1 = 0, nCrossFactor2 = 0;
	real tCrossFactor1 = 0, tCrossFactor2 = 0;

	// gradC . (mInv gradC) in normal and tangent directions
	real nMassFactor = 0;
	real tMassFactor = 0;

	// Complete set of information that specifies how this 
	// contact point was generated. Can be used to recompute
	// the contact point or to match agatinst other contact points.

	// TODO: IndexedContactPoint subclass?

	// Index of point on incident body
	int incPointIndex = -1;

	// Index of reference edge
	int refEdgeIndex = -1;

	// Index of reference point defining side-plane against which 
	// the incident point was clipped. -1 for no clip.

	// TODO: possibly redundant?
	int clippedAgainstPoint = -1;

private:

};

