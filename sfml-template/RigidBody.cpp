#include "RigidBody.h"

RigidBody::RigidBody()
{

}

void RigidBody::intVel(real dt)
{
	vel += acc * dt;
	omega += alpha * dt;

	// TODO: Apply damping?

}

void RigidBody::intPos(real dt)
{
	// Store old state for interpolation
	posPrev = pos;
	thetaPrev = theta;

	pos += vel * dt;
	theta += omega * dt;
}

vec2 RigidBody::pixCoords(real pixPerUnit) const
{
	return {pos.x * pixPerUnit, pos.y * pixPerUnit};
}
