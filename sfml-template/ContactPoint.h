#pragma once
#include "Utils.h"

struct ContactPoint
{
	bool matches(const ContactPoint& other) const;
	std::string idAsString() const;

	vec2 point;
	real penetration = 0;

	real lambda = 0;
	real fLambda = 0;
	real fRollLambda = 0;

	real vRelTarget = 0;

	// z.(r x n) and z.(r x t) for bodies 1 & 2
	real nCrossFactor1 = 0, nCrossFactor2 = 0;
	real tCrossFactor1 = 0, tCrossFactor2 = 0;

	// gradC . (mInv gradC) in normal and tangent directions
	real nMassFactor = 0;
	real tMassFactor = 0;

	// A point on the incident body used to generate the contact point
	vec2 localIncPoint;

	// Indices that specify how this contact point was generated,
	// used to match against other contact points. Only used in cases
	// where there can be more than one contact point (e.g. poly-poly).
	
	// Index of point on incident body
	int incPointIndex = -1;

	// Index of reference edge
	int refEdgeIndex = -1;
};

