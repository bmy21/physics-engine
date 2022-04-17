#include "RigidBody.h"


void RigidBody::moveTo(const vec2& p)
{
	pos = posPrev = p; 
	onMove();
}

void RigidBody::rotateTo(real t)
{
	theta = thetaPrev = t;
	onRotate();
}

void RigidBody::integrateVel(real dt)
{
	acc.y += grav;

	vel += acc * dt;
	omega += alpha * dt;

	// Reset acceleration for next frame
	acc = { 0, 0 };

	// TODO: Apply damping?

}

void RigidBody::integratePos(real dt)
{
	// Store old state for interpolation
	posPrev = pos;
	thetaPrev = theta;

	pos += vel * dt;
	theta += omega * dt;

	onMove();
	onRotate();
}

vec2 RigidBody::interpolatePos(real fraction) const
{
	// real smooth = 6 * pow(fraction, 5) - 15*pow(fraction,4)+10*pow(fraction,3);
	// fraction = smooth;

	return (1 - fraction) * posPrev + fraction * pos;
}

real RigidBody::interpolateAngle(real fraction) const
{
	vec2 dir = { std::cos(theta), std::sin(theta) };
	vec2 dirPrev = { std::cos(thetaPrev), std::sin(thetaPrev) };
	vec2 dirInterp = (1 - fraction) * dirPrev + fraction * dir;

	return std::atan2(dirInterp.y, dirInterp.x);
}
