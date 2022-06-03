#pragma once

#include "Utils.h"

struct PhysicsSettings
{
	real dt = 1.0 / 200;

	real slop = 0.005;
	real beta = 0.2;

	real vRelThreshold = 0;
	
	bool simulSolveVel = true;
	bool simulSolvePos = true;
	bool warmStart = true;

	real maxCondVel = 1000;
	real maxCondPos = 500;

	int velIter = 8;
	int posIter = 4;

	real muDefault = 0.5;
	real eDefault = 0.3;

	real grav = 10;

	real linearDamp = 0;
	real angularDamp = decayConstant(5);

	// Plane half-thickness for clipping
	real clipPlaneEpsilon = 1e-5;

	// Weighting applied in separating axis test to favour one edge over the other
	// Should be considerably less than the slop value, as for persistent contacts 
	// the separation shouldn't exceed the slop
	real refEdgeAbsTol = 5e-4;
};