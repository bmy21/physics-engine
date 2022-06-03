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

	real maxCondVel = 1000;
	real maxCondPos = 500;

	int velIter = 8;
	int posIter = 4;

	real muDefault = 0.5;
	real eDefault = 0.3;

	real grav = 10;

	real linearDamp = 0;
	real angularDamp = decayConstant(5);

	// TODO: clip plane epsilon / reference edge bias / warmStarting
};