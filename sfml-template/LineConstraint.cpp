#include "LineConstraint.h"
#include "RigidBody.h"

LineConstraint::LineConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	const vec2& localDir1, const PhysicsSettings& ps):
	rb1(rb1), rb2(rb2), localPoint1(localPoint1), localPoint2(localPoint2), localDir1(localDir1),
	Constraint(ps, { rb1, rb2 })
{
	localPerp1 = perp(localDir1);
}

void LineConstraint::correctVel()
{
	real vDotGradC = dot(rb1->velocity() - rb2->velocity(), globalPerp1) + crossFactor1 * rb1->angVel() - crossFactor2 * rb2->angVel();

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -vDotGradC / massFactor;
	}

	accLam += dLambda;

	rb1->applyDeltaVel(globalPerp1 * rb1->mInv() * dLambda, crossFactor1 * rb1->IInv() * dLambda);
	rb2->applyDeltaVel(-globalPerp1 * rb2->mInv() * dLambda, -crossFactor2 * rb2->IInv() * dLambda);
}

void LineConstraint::correctPos()
{
	updateCachedData();

	real C = dot(globalPoint1 - globalPoint2, globalPerp1);

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -ps.beta * C / massFactor;
	}

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(globalPerp1 * rb1->mInv() * dLambda, crossFactor1 * rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos(-globalPerp1 * rb2->mInv() * dLambda, -crossFactor2 * rb2->IInv() * dLambda, false);
}

void LineConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(globalPerp1 * rb1->mInv() * accLam, crossFactor1 * rb1->IInv() * accLam);
		rb2->applyDeltaVel(-globalPerp1 * rb2->mInv() * accLam, -crossFactor2 * rb2->IInv() * accLam);
	}
	else
	{
		accLam = 0;
	}
}

void LineConstraint::prepareVelSolver()
{
	updateCachedData();
}

void LineConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	globalPerp1 = rb1->vecToGlobal(localPerp1);

	real mag = magnitude(globalPerp1);

	if (mag != 0)
	{
		globalPerp1 /= mag;
	}

	crossFactor1 = zcross(globalPerp1, rb1->position() - globalPoint2);
	crossFactor2 = zcross(globalPerp1, rb2->position() - globalPoint2);

	massFactor = rb1->mInv() + rb1->IInv() * crossFactor1 * crossFactor1 + rb2->mInv() + rb2->IInv() * crossFactor2 * crossFactor2;
}
