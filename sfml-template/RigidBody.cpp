#include "RigidBody.h"

RigidBody::RigidBody(const PhysicsSettings& ps, real mInv, real IInv):
	ps(ps), mInv(mInv), IInv(IInv)
{

}

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
	if (mInv != 0)
	{
		acc.y += ps.grav;
	}

	vel += acc * dt;
	omega += alpha * dt;

	// Reset acceleration for next frame
	acc = { 0, 0 };
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
	return (1 - fraction) * posPrev + fraction * pos;
}

real RigidBody::interpolateAngle(real fraction) const
{
	vec2 dir = { std::cos(theta), std::sin(theta) };
	vec2 dirPrev = { std::cos(thetaPrev), std::sin(thetaPrev) };
	vec2 dirInterp = (1 - fraction) * dirPrev + fraction * dir;

	return std::atan2(dirInterp.y, dirInterp.x);
}

void RigidBody::applyDeltaVel(const vec2& dv, real dw)
{
	vel += dv;
	omega += dw;

	//std::cout << theta << "\n";

	assert(!isnan(dv.x));
	assert(!isnan(dv.y));
	assert(!isnan(dw));
}

void RigidBody::applyDeltaPos(const vec2& dr, real dth)
{
	// Note: previous position and angle are NOT updated here

	pos += dr;
	theta += dth;

	assert(!isnan(dr.x));
	assert(!isnan(dr.y));
	assert(!isnan(dth));

	onMove();
	onRotate();
}


void RigidBody::applyDamping(real dt)
{
	// Apply damping s.t. dv/dt = -linearDamp * v;
	// Ensure that the velocities don't have their directions reversed

	real linearScale = 1 - ps.linearDamp * dt;
	real angularScale = 1 - ps.angularDamp * dt;

	vel *= linearScale > 0 ? linearScale : 0;
	omega *= angularScale > 0 ? angularScale : 0;
}
