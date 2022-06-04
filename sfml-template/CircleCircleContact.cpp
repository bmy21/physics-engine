#include "CircleCircleContact.h"

CircleCircleContact::CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps):
	c1(c1), 
	c2(c2),
	ContactConstraint(ps)
{
	// TODO: rename & call rebuild() function here
	n = normalise(c2->position() - c1->position());

	// Place the contact point in the middle of the colliding region
	vec2 furthestPoint1 = c1->position() + n * c1->radius();
	vec2 furthestPoint2 = c2->position() - n * c2->radius();
	cp.point = static_cast<real>(0.5) * (furthestPoint1 + furthestPoint2);
	cp.penetration = dot(furthestPoint2 - furthestPoint1, n);

	real vRel = dot(c2->pointVel(cp.point) - c1->pointVel(cp.point), n);
	vRelTarget = vRel < -ps.vRelThreshold ? -e * vRel : 0;
}

void CircleCircleContact::warmStart()
{
	if (ps.warmStart)
	{
		c2->applyDeltaVel(n * c2->mInv * cp.lambda + t * c2->mInv * cp.fLambda,
			nCrossFactor2 * c2->IInv * cp.lambda + tCrossFactor2 * c2->IInv * cp.fLambda);

		c1->applyDeltaVel(-n * c1->mInv * cp.lambda - t * c1->mInv * cp.fLambda,
			-nCrossFactor1 * c1->IInv * cp.lambda - tCrossFactor1 * c1->IInv * cp.fLambda);
	}
	else
	{
		cp.lambda = cp.fLambda = 0;
	}
}

void CircleCircleContact::correctVel()
{
	// TODO: relative velocity function in ContactConstraint?
	// TODO: RigidBody pointers in ContactConstraint?
	// TODO: more general 1CP solvers in ContactConstraint?

	// Friction
	real vDotGradCf = dot(c2->velocity() - c1->velocity(), t) + tCrossFactor2 * c2->angVel() - tCrossFactor1 * c1->angVel();

	real dfLambda = 0;
	if (tMassFactor != 0)
	{
		dfLambda = -vDotGradCf / tMassFactor;
		dfLambda = std::clamp(cp.fLambda + dfLambda, -mu * cp.lambda, mu * cp.lambda) - cp.fLambda;
	}

	cp.fLambda += dfLambda;

	c2->applyDeltaVel(t * c2->mInv * dfLambda, tCrossFactor2 * c2->IInv * dfLambda);
	c1->applyDeltaVel(-t * c1->mInv * dfLambda, -tCrossFactor1 * c1->IInv * dfLambda);

	real vDotGradC = dot(c2->velocity() - c1->velocity(), n) + nCrossFactor2 * c2->angVel() - nCrossFactor1 * c1->angVel();

	real dLambda = 0;
	if (nMassFactor != 0)
	{
		dLambda = (vRelTarget - vDotGradC) / nMassFactor;
		dLambda = std::max(cp.lambda + dLambda, static_cast<real>(0)) - cp.lambda;
	}

	cp.lambda += dLambda;

	c2->applyDeltaVel(n * c2->mInv * dLambda, nCrossFactor2 * c2->IInv * dLambda);
	c1->applyDeltaVel(-n * c1->mInv * dLambda, -nCrossFactor1 * c1->IInv * dLambda);
}

void CircleCircleContact::correctPos()
{
	rebuild();
	updateCache();

	real C = std::min(cp.penetration + ps.slop, static_cast<real>(0));

	real dLambda = 0;
	if (nMassFactor != 0)
	{
		dLambda = -ps.beta * C / nMassFactor;
	}

	//std::cout << inc->mInv * dLambda << "\n";

	c2->applyDeltaPos(n * c2->mInv * dLambda, nCrossFactor2 * c2->IInv * dLambda);
	c1->applyDeltaPos(-n * c1->mInv * dLambda, -nCrossFactor1 * c1->IInv * dLambda);
}

void CircleCircleContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
}

bool CircleCircleContact::matches(const CircleCircleContact* other) const
{
	// Simply match based on colliding circles, as there can only be one contact point
	return c1 == other->c1 && c2 == other->c2;
}

void CircleCircleContact::rebuild()
{
	n = normalise(c2->position() - c1->position());

	// Place the contact point in the middle of the colliding region
	vec2 furthestPoint1 = c1->position() + n * c1->radius();
	vec2 furthestPoint2 = c2->position() - n * c2->radius();
	cp.point = static_cast<real>(0.5) * (furthestPoint1 + furthestPoint2);
	cp.penetration = dot(furthestPoint2 - furthestPoint1, n);
}

void CircleCircleContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other will be left in an invalid state

	CircleCircleContact* ccOther = static_cast<CircleCircleContact*>(other);

	cp.lambda = ccOther->cp.lambda;
	cp.fLambda = ccOther->cp.fLambda;
	vRelTarget = ccOther->vRelTarget;
}

void CircleCircleContact::updateCache()
{
	// TODO: handle zero normal case?
	// Normal points from c1 to c2
	n = normalise(c2->position() - c1->position());
	t = perp(n);

	vec2 relPos1 = cp.point - c1->position();
	vec2 relPos2 = cp.point - c2->position();

	nCrossFactor1 = zcross(relPos1, n);
	tCrossFactor1 = zcross(relPos1, t);
	nCrossFactor2 = zcross(relPos2, n);
	tCrossFactor2 = zcross(relPos2, t);

	nMassFactor = c1->mInv + c2->mInv + c1->IInv * std::pow(nCrossFactor1, 2) + c2->IInv * std::pow(nCrossFactor2, 2);
	nMassFactor = c1->mInv + c2->mInv + c1->IInv * std::pow(tCrossFactor1, 2) + c2->IInv * std::pow(tCrossFactor2, 2);
}

