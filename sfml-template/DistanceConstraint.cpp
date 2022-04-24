#include "DistanceConstraint.h"

void DistanceConstraint::correctVel()
{
	//real x = rb->position().x, y = rb->position().y;
	vec2 pos = rb->position(); 
	vec2 vel = rb->velocity();

	real vDotGradC = dot(vel, pos - point) + zcross({0, 0}, pos - point);
	real massFactor = rb->mInv * (std::pow(pos.x - point.x, 2) + std::pow(pos.y - point.y, 2)) 
		+ rb->IInv * std::pow(zcross(pos - point, {0, 0}), 2);

	real lambda = -vDotGradC / massFactor;
	vec2 dv = lambda * (pos - point);
	real dw = lambda * zcross({ 0, 0 }, pos - point);

	rb->applyDeltaVel(dv, dw);
}

void DistanceConstraint::correctPos()
{
	vec2 pos = rb->position();

	real massFactor = rb->mInv * (std::pow(pos.x - point.x, 2) + std::pow(pos.y - point.y, 2))
		+ rb->IInv * std::pow(zcross(pos - point, { 0, 0 }), 2);

	real C = dot(pos - point, pos - point);
	real beta = 0.4;

	real lambda = -beta * C / massFactor;
	vec2 dr = lambda * (pos - point);
	real dth = lambda * zcross({ 0, 0 }, pos - point);

	rb->applyDeltaPos(dr, dth);
}
