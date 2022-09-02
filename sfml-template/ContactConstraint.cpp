#include "ContactConstraint.h"

ContactConstraint::ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2):
	ps(ps), rb1(rb1), rb2(rb2)
{
	
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
	
	rb1->applyDeltaVel(-n * rb1->mInv() * dLambda, -cp.nCrossFactor1 * rb1->IInv() * dLambda);
	rb2->applyDeltaVel(n * rb2->mInv() * dLambda, cp.nCrossFactor2 * rb2->IInv() * dLambda);
}

void ContactConstraint::solvePointPos(ContactPoint& cp)
{
	//real maxCorr = 0.1;
	real C = std::min(cp.penetration + ps.slop, static_cast<real>(0));

	//float C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0f);


	real dLambda = 0;
	if (cp.nMassFactor != 0)
	{
		dLambda = -ps.beta * C / cp.nMassFactor;
	}

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(-n * rb1->mInv() * dLambda, -cp.nCrossFactor1 * rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos(n * rb2->mInv() * dLambda, cp.nCrossFactor2 * rb2->IInv() * dLambda, false);
}

void ContactConstraint::warmStartPoint(ContactPoint& cp)
{
	if (ps.warmStart)
	{
		// Don't warm start rolling friction - this occasionally causes infinite spinning and should be investigated!
		cp.fRollLambda = 0;

		rb1->applyDeltaVel(-n * rb1->mInv() * cp.lambda - t * rb1->mInv() * cp.fLambda,
			-cp.nCrossFactor1 * rb1->IInv() * cp.lambda - cp.tCrossFactor1 * rb1->IInv() * cp.fLambda - rb1->IInv() * cp.fRollLambda);

		rb2->applyDeltaVel(n * rb2->mInv() * cp.lambda + t * rb2->mInv() * cp.fLambda,
			cp.nCrossFactor2 * rb2->IInv() * cp.lambda + cp.tCrossFactor2 * rb2->IInv() * cp.fLambda + rb2->IInv() * cp.fRollLambda);
	}
	else
	{
		cp.lambda = cp.fLambda = cp.fRollLambda = 0;
	}
}

bool ContactConstraint::idsMatch(const ContactConstraint* other) const
{
	return rb1->id == other->rb1->id && rb2->id == other->rb2->id;
}

void ContactConstraint::storeTargetVelocities()
{
	updateNormal();

	for (auto& cp : contactPoints)
	{
		real vRel = dot(rb2->pointVel(cp.point) - rb1->pointVel(cp.point), n);
		cp.vRelTarget = vRel < -ps.vRelThreshold ? -e * vRel : 0;
	}
}

void ContactConstraint::updateTangent()
{
	t = perp(n);
}

void ContactConstraint::prepareSimulSolver()
{
	ContactPoint& cp1 = contactPoints[0];
	ContactPoint& cp2 = contactPoints[1];

	A12 = rb2->mInv() + rb1->mInv() + rb2->IInv() * cp1.nCrossFactor2 * cp2.nCrossFactor2 + rb1->IInv() * cp1.nCrossFactor1 * cp2.nCrossFactor1;
	det = cp1.nMassFactor * cp2.nMassFactor - A12 * A12;
	norm = std::max(cp1.nMassFactor, cp2.nMassFactor) + std::abs(A12);
	real normSquared = norm * norm;

	// Is the condition number less than the threshold?
	wellConditionedVel = normSquared < ps.maxCondVel * det;
	wellConditionedPos = normSquared < ps.maxCondPos * det;
}

bool ContactConstraint::simulSolveVel()
{
	ContactPoint& cp1 = contactPoints[0];
	ContactPoint& cp2 = contactPoints[1];

	real alpha1 = cp1.vRelTarget - (dot(rb2->velocity() - rb1->velocity(), n) + cp1.nCrossFactor2 * rb2->angVel() - cp1.nCrossFactor1 * rb1->angVel());
	real alpha2 = cp2.vRelTarget - (dot(rb2->velocity() - rb1->velocity(), n) + cp2.nCrossFactor2 * rb2->angVel() - cp2.nCrossFactor1 * rb1->angVel());
	

	// First assume both accumulated impulses are non-negative
	real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
	real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;

	if (cp1.lambda + lam1 >= 0 && cp2.lambda + lam2 >= 0)
	{
		rb1->applyDeltaVel(-n * rb1->mInv() * (lam1 + lam2), rb1->IInv() * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
		rb2->applyDeltaVel(n * rb2->mInv() * (lam1 + lam2), rb2->IInv() * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

		cp1.lambda += lam1;
		cp2.lambda += lam2;

		return true;
	}

	// Now try assuming both accumulated impulses are zero
	// (Avoid the asymmetrical solutions if possible)
	lam1 = -cp1.lambda;
	lam2 = -cp2.lambda;

	if (lam1 * cp1.nMassFactor + lam2 * A12 >= alpha1 && lam1 * A12 + lam2 * cp2.nMassFactor >= alpha2)
	{
		rb1->applyDeltaVel(-n * rb1->mInv() * (lam1 + lam2), rb1->IInv() * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
		rb2->applyDeltaVel(n * rb2->mInv() * (lam1 + lam2), rb2->IInv() * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

		cp1.lambda += lam1;
		cp2.lambda += lam2;

		return true;
	}

	// Now try assuming accumulated impulse 1 is zero and accumulated impulse 2 is non-negative
	lam1 = -cp1.lambda;
	lam2 = alpha2 / cp2.nMassFactor;

	if (cp2.lambda + lam2 >= 0 && lam1 * cp1.nMassFactor + lam2 * A12 >= alpha1)
	{
		rb1->applyDeltaVel(-n * rb1->mInv() * (lam1 + lam2), rb1->IInv() * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
		rb2->applyDeltaVel(n * rb2->mInv() * (lam1 + lam2), rb2->IInv() * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

		cp1.lambda += lam1;
		cp2.lambda += lam2;

		return true;
	}

	// Now try assuming accumulated impulse 2 is zero and accumulated impulse 1 is non-negative
	lam1 = alpha1 / cp1.nMassFactor;
	lam2 = -cp2.lambda;

	if (cp1.lambda + lam1 >= 0 && lam2 * cp2.nMassFactor + lam1 * A12 >= alpha2)
	{
		rb1->applyDeltaVel(-n * rb1->mInv() * (lam1 + lam2), rb1->IInv() * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
		rb2->applyDeltaVel(n * rb2->mInv() * (lam1 + lam2), rb2->IInv() * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

		cp1.lambda += lam1;
		cp2.lambda += lam2;

		return true;
	}
	

	return false;
}

void ContactConstraint::simulSolvePos()
{
	ContactPoint& cp1 = contactPoints[0];
	ContactPoint& cp2 = contactPoints[1];

	real C1 = std::min(cp1.penetration + ps.slop, static_cast<real>(0));
	real C2 = std::min(cp2.penetration + ps.slop, static_cast<real>(0));

	real alpha1 = -ps.beta * C1;
	real alpha2 = -ps.beta * C2;

	real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
	real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(-n * rb1->mInv() * (lam1 + lam2), rb1->IInv() * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2), false);
	rb2->applyDeltaPos(n * rb2->mInv() * (lam1 + lam2), rb2->IInv() * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2), false);
}

void ContactConstraint::updateNormalFactors(ContactPoint& cp)
{
	vec2 relPos1 = cp.point - rb1->position();
	vec2 relPos2 = cp.point - rb2->position();

	cp.nCrossFactor1 = zcross(relPos1, n);
	cp.nCrossFactor2 = zcross(relPos2, n);
	cp.nMassFactor = rb1->mInv() + rb2->mInv() + rb1->IInv() * std::pow(cp.nCrossFactor1, 2) + rb2->IInv() * std::pow(cp.nCrossFactor2, 2);
}

void ContactConstraint::updateTangentFactors(ContactPoint& cp)
{
	vec2 relPos1 = cp.point - rb1->position();
	vec2 relPos2 = cp.point - rb2->position();

	cp.tCrossFactor1 = zcross(relPos1, t);
	cp.tCrossFactor2 = zcross(relPos2, t);
	cp.tMassFactor = rb1->mInv() + rb2->mInv() + rb1->IInv() * std::pow(cp.tCrossFactor1, 2) + rb2->IInv() * std::pow(cp.tCrossFactor2, 2);
}
