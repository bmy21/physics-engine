#include "ContactConstraint.h"

ContactConstraint::ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2):
	ps(ps), rb1(rb1), rb2(rb2)
{
	e = ps.eDefault;
	mu = ps.muDefault;

	// Sort by index of incident point to ensure a consistent ordering
	// Note - clipping process above should give consistent order anyway?
	// std::sort(contactPoints.begin(), contactPoints.end(),
	//	[](const ContactPoint& cp1, const ContactPoint& cp2) 
	//	{ return cp1.incPointIndex < cp2.incPointIndex; }); 
}

void ContactConstraint::init()
{
	initPoints();
	ncp = contactPoints.size();
	storeTargetVelocities();
}

void ContactConstraint::correctVel()
{
	for (auto& cp : contactPoints)
	{
		solvePointFriction(cp);
	}

	// Try simultaneous solution of normal velocities first
	if (ncp == 2 && ps.simulSolveVel && wellConditionedVel)
	{
		ContactPoint& cp1 = contactPoints[0];
		ContactPoint& cp2 = contactPoints[1];

		real alpha1 = cp1.vRelTarget - (dot(rb2->velocity() - rb1->velocity(), n) + cp1.nCrossFactor2 * rb2->angVel() - cp1.nCrossFactor1 * rb1->angVel());
		real alpha2 = cp2.vRelTarget - (dot(rb2->velocity() - rb1->velocity(), n) + cp2.nCrossFactor2 * rb2->angVel() - cp2.nCrossFactor1 * rb1->angVel());

		real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
		real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;


		if (cp1.lambda + lam1 >= 0 && cp2.lambda + lam2 >= 0)
		{
			rb1->applyDeltaVel(-n * rb1->mInv * (lam1 + lam2), rb1->IInv * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
			rb2->applyDeltaVel(n * rb2->mInv * (lam1 + lam2), rb2->IInv * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

			cp1.lambda += lam1;
			cp2.lambda += lam2;

			return;
		}
	}

	// At this point, simultaneous solution failed, either because the condition number was too high or 
	// because one of the accumulated impulses would become negative. So, resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		solvePointVel(cp);
	}
}

void ContactConstraint::correctPos()
{
	// Try simultaneous solution first
	if (ncp == 2 && ps.simulSolvePos)
	{
		updateNormal();

		for (auto& cp : contactPoints)
		{
			rebuildPoint(cp);
			updateNormalFactors(cp);
		}

		prepareSimulSolver();

		if (wellConditionedPos)
		{
			ContactPoint& cp1 = contactPoints[0];
			ContactPoint& cp2 = contactPoints[1];

			real C1 = std::min(cp1.penetration + ps.slop, static_cast<real>(0));
			real C2 = std::min(cp2.penetration + ps.slop, static_cast<real>(0));

			real alpha1 = -ps.beta * C1;
			real alpha2 = -ps.beta * C2;

			real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
			real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;

			rb1->applyDeltaPos(-n * rb1->mInv * (lam1 + lam2), rb1->IInv * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));
			rb2->applyDeltaPos(n * rb2->mInv * (lam1 + lam2), rb2->IInv * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));

			return;
		}
	}

	// At this point, the condition number was too high, so resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		updateNormal();
		rebuildPoint(cp);
		updateNormalFactors(cp);
		solvePointPos(cp);
	}
}

void ContactConstraint::warmStart()
{
	for (auto& cp : contactPoints)
	{
		warmStartPoint(cp);
	}
}

void ContactConstraint::prepareVelSolver()
{
	updateNormal();
	updateTangent();

	for (auto& cp : contactPoints)
	{
		updateNormalFactors(cp);
		updateTangentFactors(cp);
	}

	if (ncp == 2 && ps.simulSolveVel)
	{
		prepareSimulSolver();
	}
}

void ContactConstraint::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other will be left in an invalid state

	for (int i = 0; i < ncp; ++i)
	{
		// Transfer accumulated impulses to new constraint
		other->contactPoints[i].lambda = contactPoints[i].lambda;
		other->contactPoints[i].fLambda = contactPoints[i].fLambda;
	}

	contactPoints = std::move(other->contactPoints);

	onRebuildFrom(other);
}

void ContactConstraint::solvePointFriction(ContactPoint& cp)
{
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

	A12 = rb2->mInv + rb1->mInv + rb2->IInv * cp1.nCrossFactor2 * cp2.nCrossFactor2 + rb1->IInv * cp1.nCrossFactor1 * cp2.nCrossFactor1;
	det = cp1.nMassFactor * cp2.nMassFactor - A12 * A12;
	norm = std::max(cp1.nMassFactor, cp2.nMassFactor) + std::abs(A12);
	real normSquared = norm * norm;

	// Is the condition number less than the threshold?
	wellConditionedVel = normSquared < ps.maxCondVel * det;
	wellConditionedPos = normSquared < ps.maxCondPos * det;
}

void ContactConstraint::updateNormalFactors(ContactPoint& cp)
{
	vec2 relPos1 = cp.point - rb1->position();
	vec2 relPos2 = cp.point - rb2->position();

	cp.nCrossFactor1 = zcross(relPos1, n);
	cp.nCrossFactor2 = zcross(relPos2, n);
	cp.nMassFactor = rb1->mInv + rb2->mInv + rb1->IInv * std::pow(cp.nCrossFactor1, 2) + rb2->IInv * std::pow(cp.nCrossFactor2, 2);
}

void ContactConstraint::updateTangentFactors(ContactPoint& cp)
{
	vec2 relPos1 = cp.point - rb1->position();
	vec2 relPos2 = cp.point - rb2->position();

	cp.tCrossFactor1 = zcross(relPos1, t);
	cp.tCrossFactor2 = zcross(relPos2, t);
	cp.tMassFactor = rb1->mInv + rb2->mInv + rb1->IInv * std::pow(cp.tCrossFactor1, 2) + rb2->IInv * std::pow(cp.tCrossFactor2, 2);
}
