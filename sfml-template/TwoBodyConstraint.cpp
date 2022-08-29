#include "TwoBodyConstraint.h"
#include "RigidBody.h"

TwoBodyConstraint::TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, const PhysicsSettings& ps):
	rb1(rb1), rb2(rb2),
	Constraint(ps, { rb1, rb2 })
{

}

TwoBodyConstraint::TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, real tOsc, real dampingRatio, const PhysicsSettings& ps):
	TwoBodyConstraint(rb1, rb2, ps)
{
	makeSpringy(tOsc, dampingRatio);
}

void TwoBodyConstraint::makeSpringy(real tOsc, real dampingRatio)
{
	updateCachedData();

	if (massFactor == 0)
	{
		// Cannot soften the constraint if the effective mass is infinite
		makeRigid();
	}
	else
	{
		isRigid = false;

		real k = 4 * pi * pi / (tOsc * tOsc * massFactor);
		real b = dampingRatio * 2 * std::sqrt(k / massFactor);

		real denom = ps.dt * k + b;

		beta = k / denom;
		gamma = 1 / (ps.dt * denom);
	}
}

void TwoBodyConstraint::makeRigid()
{
	isRigid = true;
	beta = gamma = 0;
}

void TwoBodyConstraint::setAsDamper(real tDamp)
{
	isRigid = false;
	beta = 0;
	real b = decayConstant(tDamp);
	gamma = 1 / (ps.dt * b);
}

void TwoBodyConstraint::enableMotor(real vTarget, real fMax)
{
	motorEnabled = true;
	motorTarget = vTarget;
	motorMaxForce = fMax;
}

void TwoBodyConstraint::disableMotor()
{
	motorEnabled = false;
}

void TwoBodyConstraint::setTarget(real t)
{
	C0 = t;
}

void TwoBodyConstraint::setRange(real small, real large)
{
	isLimited = true;
	Cmin = small;
	Cmax = large;
}

void TwoBodyConstraint::removeLimits()
{
	isLimited = false;
}

void TwoBodyConstraint::correctVel()
{
	if (motorEnabled && massFactor != 0)
	{
		real maxImpulse = motorMaxForce * ps.dt;
		real dLambda = (motorTarget - getvDotGradC()) / massFactor;
		dLambda = std::clamp(accMotor + dLambda, -maxImpulse, maxImpulse) - accMotor;
		accMotor += dLambda;
		applyImpulse(dLambda);
	}

	if (!(isRigid && isLimited) && !(isRigid && motorEnabled) && (massFactor + gamma != 0))
	{
		// Apply corrective impulse unless the constraint is a rigid slider
		real dLambda = -(getvDotGradC() + beta * (C - C0) + gamma * accLam) / (massFactor + gamma);
		accLam += dLambda;
		applyImpulse(dLambda);
	}

	
	if (isLimited && massFactor != 0)
	{
		// Apply rigid inequality constraints to keep the constraint function within the limits

		// First, ensure C > Cmin
		real targetVel = 0;

		if (C > Cmin)
		{
			// Try to change the velocity such that C = Cmin after the impulse
			targetVel = (Cmin - C) / ps.dt;
		}

		// If C <= Cmin already, try to reduce the velocity to zero
		real dLambda = (targetVel - getvDotGradC()) / massFactor;

		// Only allow the impulse if it would push away (i.e. is positive)
		dLambda = std::max(accLower + dLambda, static_cast<real>(0)) - accLower;
		applyImpulse(dLambda);
	

		// Now ensure C < Cmax
		targetVel = 0;

		if (C < Cmax)
		{
			targetVel = (Cmax - C) / ps.dt;
		}

		dLambda = (targetVel - getvDotGradC()) / massFactor;

		// Upper impulse must push in the opposite direction (i.e. is negative)
		dLambda = std::min(accLower + dLambda, static_cast<real>(0)) - accLower;
		applyImpulse(dLambda);
	}
}

void TwoBodyConstraint::correctPos()
{
	updateCachedData();

	if (massFactor == 0)
	{
		return;
	}

	real dLambda = 0;
	real numerator = 0;

	if (isLimited)
	{
		// Aim for the lower or upper limit as appropriate
		if (C > Cmax)
		{
			numerator = C - Cmax;
		}
		else if (C < Cmin)
		{
			numerator = C - Cmin;
		}
	}
	else if (isRigid && !motorEnabled)
	{
		// Aim for C0
		numerator = C - C0;
	}
	else
	{
		// A free spring - no position correction needed
	}

	dLambda = -ps.beta * numerator / massFactor;

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(vec2(gradC[0], gradC[1]) * rb1->mInv() * dLambda, gradC[2] * rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos(vec2(gradC[3], gradC[4]) * rb2->mInv() * dLambda, gradC[5] * rb2->IInv() * dLambda, false);
}

void TwoBodyConstraint::warmStart()
{
	if (ps.warmStart)
	{
		applyImpulse(accLam + accLower + accUpper + accMotor);
	}
	else
	{
		accLam = 0;
		accLower = 0;
		accUpper = 0;
		accMotor = 0;
	}
}

void TwoBodyConstraint::prepareVelSolver()
{
	updateCachedData();
}

void TwoBodyConstraint::applyImpulse(real impulse)
{
	rb1->applyDeltaVel(vec2(gradC[0], gradC[1]) * rb1->mInv() * impulse, gradC[2] * rb1->IInv() * impulse);
	rb2->applyDeltaVel(vec2(gradC[3], gradC[4]) * rb2->mInv() * impulse, gradC[5] * rb2->IInv() * impulse);
}

real TwoBodyConstraint::getvDotGradC() const
{
	vec2 v1 = rb1->velocity(), v2 = rb2->velocity();
	real w1 = rb1->angVel(), w2 = rb2->angVel();

	return v1.x * gradC[0] + v1.y * gradC[1] + w1 * gradC[2] + v2.x * gradC[3] + v2.y * gradC[4] + w2 * gradC[5];
}
