#pragma once

#include "Utils.h"

struct PhysicsSettings
{
	real dt = 1.0 / 200;

	real slop = 0.005;
	real beta = 0.1;

	real vRelThreshold = 0.1;
	
	bool simulSolveVel = true;
	bool simulSolvePos = 0;
	bool warmStart = 1;  

	real maxCondVel = 1000;
	real maxCondPos = 500;

	int maxIterGJK = 50;

	int velIter = 12;
	int posIter = 0;

	real grav = 10;

	real muDefault = 0.5;
	real eDefault = 0.2;

	// TODO: add AABB fattening width

	// Perpendicular distance of normal reaction from point of contact 
	// used to compute rolling friction torque
	real rfLengthDefault = 0.01;

	// Decay constants for all rigid bodies
	real linearDamp = 0;
	real angularDamp = 0;

	// Plane half-thickness for clipping
	real clipPlaneEpsilon = 1e-5;

	// Weighting applied in separating axis test to favour one edge over the other
	// Should be considerably less than the slop value, as for persistent contacts 
	// the separation shouldn't exceed the slop
	real refEdgeAbsTol = 5e-4;
};