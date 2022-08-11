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
	// NOTE: effective mass may change during motion due to rotation 
	// TODO: allow caller to supply an alternative effective mass?

	updateCachedData();

	if (massFactor == 0)
	{
		// Cannot soften the constraint if the effective mass is infinite
		makeRigid();
	}
	else
	{
		shouldCorrectPos = false;

		real k = 4 * pi * pi / (tOsc * tOsc * massFactor);
		real b = dampingRatio * 2 * std::sqrt(k / massFactor);

		real denom = ps.dt * k + b;

		beta = k / denom;
		gamma = 1 / (ps.dt * denom);
		//std::cout << massFactor << " /// " << beta << " /// " << gamma << "\n";
	}
}

void TwoBodyConstraint::makeRigid()
{
	shouldCorrectPos = true;
	beta = gamma = 0;
}

void TwoBodyConstraint::correctVel()
{
	vec2 v1 = rb1->velocity(), v2 = rb2->velocity();
	real w1 = rb1->angVel(), w2 = rb2->angVel();

	real vDotGradC = v1.x * gradC[0] + v1.y * gradC[1] + w1 * gradC[2] + v2.x * gradC[3] + v2.y * gradC[4] + w2 * gradC[5];

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -(vDotGradC + beta * C + gamma * accLam) / (massFactor + gamma);
	}

	//std::cout << dLambda << "\n";
	accLam += dLambda;

	rb1->applyDeltaVel(vec2(gradC[0], gradC[1]) * rb1->mInv() * dLambda, gradC[2] * rb1->IInv() * dLambda);
	rb2->applyDeltaVel(vec2(gradC[3], gradC[4]) * rb2->mInv() * dLambda, gradC[5] * rb2->IInv() * dLambda);
}

void TwoBodyConstraint::correctPos()
{
	if (!shouldCorrectPos)
	{
		return;
	}

	updateCachedData();

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -ps.beta * C / massFactor;
	}

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos(vec2(gradC[0], gradC[1]) * rb1->mInv() * dLambda, gradC[2] * rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos(vec2(gradC[3], gradC[4]) * rb2->mInv() * dLambda, gradC[5] * rb2->IInv() * dLambda, false);
}

void TwoBodyConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(vec2(gradC[0], gradC[1]) * rb1->mInv() * accLam, gradC[2] * rb1->IInv() * accLam);
		rb2->applyDeltaVel(vec2(gradC[3], gradC[4]) * rb2->mInv() * accLam, gradC[5] * rb2->IInv() * accLam);
	}
	else
	{
		accLam = 0;
	}
}

void TwoBodyConstraint::prepareVelSolver()
{
	updateCachedData();
}
