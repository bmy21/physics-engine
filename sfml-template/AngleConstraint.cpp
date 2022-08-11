#include "AngleConstraint.h"
#include "RigidBody.h"

AngleConstraint::AngleConstraint(RigidBody* rb1, RigidBody* rb2, real angleDiff, const PhysicsSettings& ps):
	rb1(rb1), rb2(rb2), angleDiff(angleDiff),
	Constraint(ps, { rb1, rb2 })
{

}

void AngleConstraint::correctVel()
{
	real vDotGradC = rb1->angVel() - rb2->angVel();

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -vDotGradC / massFactor;
	}

	accLam += dLambda;

	rb1->applyDeltaVel({ 0, 0 }, rb1->IInv() * dLambda);
	rb2->applyDeltaVel({ 0, 0 }, -rb2->IInv() * dLambda);
}

void AngleConstraint::correctPos()
{
	updateCachedData();

	real C = rb1->angle() - rb2->angle() - angleDiff;

	real dLambda = 0;
	if (massFactor != 0)
	{
		dLambda = -ps.beta * C / massFactor;
	}

	// Don't need to call the RigidBody update functions until after the iterations are complete
	rb1->applyDeltaPos({ 0, 0 }, rb1->IInv() * dLambda, false);
	rb2->applyDeltaPos({ 0, 0 }, -rb2->IInv() * dLambda, false);
}

void AngleConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel({ 0, 0 }, rb1->IInv() * accLam);
		rb2->applyDeltaVel({ 0, 0 }, -rb2->IInv() * accLam);
	}
	else
	{
		accLam = 0;
	}
}

void AngleConstraint::prepareVelSolver()
{
	updateCachedData();
}

void AngleConstraint::updateCachedData()
{
	massFactor = rb1->IInv() + rb2->IInv();
}
