#pragma once

struct CarDefinition
{
	real bodyWidth, bodyHeight;
	real bodymInv;

	real susptOsc;
	real suspDamp;
	real suspFracChange;

	real targetAngVel;
	real maxTorque;

	std::vector<real> wheelRad;
	std::vector<real> wheelmInv;

	std::vector<vec2> wheelPos;
	std::vector<vec2> suspTop;
};