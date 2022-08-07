#pragma once

#include "Utils.h"

struct PhysicsSettings
{
	real dt = 1.0 / 120;

	bool showContactConstraints = false;
	bool showAABBTree = false;

	real slop = 0.005;
	real beta = 0.2;

	real vRelThreshold = 0.1;

	bool simulSolveVel = true;
	bool simulSolvePos = false;
	bool warmStart = true;

	real maxCondVel = 5000; 
	real maxCondPos = 500;

	int maxIterGJK = 30;

	int velIter = 12;
	int posIter = 3;

	real grav = 12;

	real muDefault = 0.5;
	real eDefault = 0.2;

	real aabbFattening = 0.1;

	// 1 Physics unit = pixPerUnit pixels
	real pixPerUnit = 120;

	// Perpendicular distance of normal reaction from point of contact 
	// used to compute rolling friction torque
	real rfLengthDefault = 0.01;

	// Decay constants for all rigid bodies
	real linearDamp = 0;
	real angularDamp = decayConstant(3);

	// Plane half-thickness for clipping
	real clipPlaneEpsilon = 1e-5;

	// Weighting applied in separating axis test to favour one edge over the other
	// Should be considerably less than the slop value, as for persistent contacts 
	// the separation shouldn't exceed the slop
	real refEdgeAbsTol = 5e-4;
};