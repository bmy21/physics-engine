#include "DistanceConstraint.h"

DistanceConstraint::DistanceConstraint()
{
	setBeta(0.1);
}

void DistanceConstraint::correctVel()
{
	vec2 pos = rb->position(); 
	vec2 vel = rb->velocity();

	real vDotGradC = dot(vel, pos - point) + zcross({0, 0}, pos - point);
	real massFactor = rb->mInv * (std::pow(pos.x - point.x, 2) + std::pow(pos.y - point.y, 2)) 
		+ rb->IInv * std::pow(zcross(pos - point, {0, 0}), 2);

	//std::cout << massFactor << '\n';

	real lambda = 0;

	if (massFactor != 0)
	{
		lambda = -vDotGradC / massFactor;
	}
	
	vec2 dv = lambda * (pos - point);
	real dw = lambda * zcross({ 0, 0 }, pos - point);

	storedLambda = lambda;

	rb->applyDeltaVel(dv, dw);
}

void DistanceConstraint::correctPos()
{
	vec2 pos = rb->position();

	real massFactor = rb->mInv * (std::pow(pos.x - point.x, 2) + std::pow(pos.y - point.y, 2))
		+ rb->IInv * std::pow(zcross(pos - point, { 0, 0 }), 2);

	real C = dot(pos - point, pos - point);

	real lambda = 0;

	if (massFactor != 0)
	{
		lambda = -beta() * C / massFactor;
	}

	vec2 dr = lambda * (pos - point);
	real dth = lambda * zcross({ 0, 0 }, pos - point);

	rb->applyDeltaPos(dr, dth);
}

void DistanceConstraint::warmStart()
{
	/*real lambda = storedLambda;
	vec2 pos = rb->position();

	vec2 dv = lambda * (pos - point);
	real dw = lambda * zcross({ 0, 0 }, pos - point);

	rb->applyDeltaVel(dv, dw);*/
}
