#include "ContactConstraint.h"

ContactConstraint::ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2):
	ps(ps), rb1(rb1), rb2(rb2)
{
	e = ps.eDefault;
	mu = ps.muDefault;
}

void ContactConstraint::solvePointFriction(ContactPoint& cp)
{
	// TODO: relative velocity function in ContactConstraint?
	real vDotGradCf = dot(rb2->velocity() - rb1->velocity(), t) + cp.tCrossFactor2 * rb2->angVel() - cp.tCrossFactor1 * rb1->angVel();

	real dfLambda = 0;
	if (cp.tMassFactor != 0)
	{
		dfLambda = -vDotGradCf / cp.tMassFactor;
		dfLambda = std::clamp(cp.fLambda + dfLambda, -mu * cp.lambda, mu * cp.lambda) - cp.fLambda;
	}

	cp.fLambda += dfLambda;

	rb1->applyDeltaVel(-t * rb1->mInv * dfLambda, -cp.tCrossFactor1 * rb1->IInv * dfLambda);
	rb2->applyDeltaVel(t * rb2->mInv * dfLambda, cp.tCrossFactor2 * rb2->IInv * dfLambda);
}

void ContactConstraint::solvePointVel(ContactPoint& cp)
{
	real vDotGradC = dot(rb2->velocity() - rb1->velocity(), n) + cp.nCrossFactor2 * rb2->angVel() - cp.nCrossFactor1 * rb1->angVel();
	
	real dLambda = 0;
	if (cp.nMassFactor != 0)
	{
		dLambda = (cp.vRelTarget - vDotGradC) / cp.nMassFactor;
		dLambda = std::max(cp.lambda + dLambda, static_cast<real>(0)) - cp.lambda;
	}

	cp.lambda += dLambda;
	
	rb1->applyDeltaVel(-n * rb1->mInv * dLambda, -cp.nCrossFactor1 * rb1->IInv * dLambda);
	rb2->applyDeltaVel(n * rb2->mInv * dLambda, cp.nCrossFactor2 * rb2->IInv * dLambda);
}

void ContactConstraint::solvePointPos(ContactPoint& cp)
{
	rebuild();
	updateCache(); // TODO: only need to update some (normal) parts of the cache here...

	real C = std::min(cp.penetration + ps.slop, static_cast<real>(0));

	real dLambda = 0;
	if (cp.nMassFactor != 0)
	{
		dLambda = -ps.beta * C / cp.nMassFactor;
	}

	rb1->applyDeltaPos(-n * rb1->mInv * dLambda, -cp.nCrossFactor1 * rb1->IInv * dLambda);
	rb2->applyDeltaPos(n * rb2->mInv * dLambda, cp.nCrossFactor2 * rb2->IInv * dLambda);
}

void ContactConstraint::warmStartPoint(ContactPoint& cp)
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(-n * rb1->mInv * cp.lambda - t * rb1->mInv * cp.fLambda,
			-cp.nCrossFactor1 * rb1->IInv * cp.lambda - cp.tCrossFactor1 * rb1->IInv * cp.fLambda);

		rb2->applyDeltaVel(n * rb2->mInv * cp.lambda + t * rb2->mInv * cp.fLambda,
			cp.nCrossFactor2 * rb2->IInv * cp.lambda + cp.tCrossFactor2 * rb2->IInv * cp.fLambda);
	}
	else
	{
		cp.lambda = cp.fLambda = 0;
	}
}

void ContactConstraint::updatePointCache(ContactPoint& cp)
{
	vec2 relPos1 = cp.point - rb1->position();
	vec2 relPos2 = cp.point - rb2->position();

	cp.nCrossFactor1 = zcross(relPos1, n);
	cp.tCrossFactor1 = zcross(relPos1, t);
	cp.nCrossFactor2 = zcross(relPos2, n);
	cp.tCrossFactor2 = zcross(relPos2, t);

	cp.nMassFactor = rb1->mInv + rb2->mInv + rb1->IInv * std::pow(cp.nCrossFactor1, 2) + rb2->IInv * std::pow(cp.nCrossFactor2, 2);
	cp.tMassFactor = rb1->mInv + rb2->mInv + rb1->IInv * std::pow(cp.tCrossFactor1, 2) + rb2->IInv * std::pow(cp.tCrossFactor2, 2);
}
