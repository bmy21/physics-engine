#include "DistanceConstraint.h"
#include "RigidBody.h"

DistanceConstraint::DistanceConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	real dist, const PhysicsSettings& ps):
	rb1(rb1), rb2(rb2), localPoint1(localPoint1), localPoint2(localPoint2), dist(dist),
	Constraint(ps, { rb1, rb2 })
{

}

void DistanceConstraint::correctVel()
{
	real vDotGradC = dot(rb1->velocity() - rb2->velocity(), n) + crossFactor1 * rb1->angVel() - crossFactor2 * rb2->angVel();

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = - vDotGradC / massFactor;
	}

	accLam += dLambda;

	rb1->applyDeltaVel(n * rb1->mInv() * dLambda, crossFactor1 * rb1->IInv() * dLambda);
	rb2->applyDeltaVel(-n * rb2->mInv() * dLambda, -crossFactor2 * rb2->IInv() * dLambda);
}

void DistanceConstraint::correctPos()
{
	updateCachedData();

	real C = magnitude(globalPoint1 - globalPoint2) - dist;

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -ps.beta * C / massFactor;
	}

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(n * rb1->mInv() * dLambda, crossFactor1 * rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos(-n * rb2->mInv() * dLambda, crossFactor2 * rb2->IInv() * dLambda, false);
}

void DistanceConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(n * rb1->mInv() * accLam, crossFactor1 * rb1->IInv() * accLam);
		rb2->applyDeltaVel(-n * rb2->mInv() * accLam, -crossFactor2 * rb2->IInv() * accLam);
	}
	else
	{
		accLam = 0;
	}
}

void DistanceConstraint::prepareVelSolver()
{
	updateCachedData();
}

void DistanceConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	n = globalPoint1 - globalPoint2;

	real mag = magnitude(n);

	if (mag != 0)
	{
		n /= mag;
	}

	crossFactor1 = zcross(globalPoint1 - rb1->position(), n);
	crossFactor2 = zcross(globalPoint2 - rb2->position(), n);

	massFactor = rb1->mInv() + rb1->IInv() * crossFactor1 * crossFactor1 + rb2->mInv() + rb2->IInv() * crossFactor2 * crossFactor2;
}
