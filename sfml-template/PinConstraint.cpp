#include "PinConstraint.h"
#include "RigidBody.h"

PinConstraint::PinConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, 
	const PhysicsSettings& ps, bool relativeToRefPoints):
	rb1(rb1), rb2(rb2), localPoint1(localPoint1), localPoint2(localPoint2),
	xhat({1, 0}), yhat({0, 1}),
	Constraint(ps, { rb1, rb2 })
{
	if (relativeToRefPoints)
	{
		this->localPoint1 += rb1->getRefPoint();
		this->localPoint2 += rb2->getRefPoint();
	}
}

void PinConstraint::correctVel()
{
	real vDotGradC1 = rb1->velocity().x + rb1->angVel() * d1crossx - rb2->velocity().x - rb2->angVel() * d2crossx;
	real vDotGradC2 = rb1->velocity().y + rb1->angVel() * d1crossy - rb2->velocity().y - rb2->angVel() * d2crossy;
	
	if (det != 0)
	{
		real target1 = -vDotGradC1, target2 = -vDotGradC2;
		real lam1 = (A22 * target1 - A12 * target2) / det;
		real lam2 = (A11 * target2 - A12 * target1) / det;

		accLam1 += lam1;
		accLam2 += lam2;

		rb1->applyDeltaVel(rb1->mInv() * (xhat * lam1 + yhat * lam2), rb1->IInv() * (d1crossx * lam1 + d1crossy * lam2));
		rb2->applyDeltaVel(-rb2->mInv() * (xhat * lam1 + yhat * lam2), -rb2->IInv() * (d2crossx * lam1 + d2crossy * lam2));
	}
}

void PinConstraint::correctPos()
{
	updateCachedData();

	if (det != 0)
	{
		real target1 = -ps.beta * C1, target2 = -ps.beta * C2;
		real lam1 = (A22 * target1 - A12 * target2) / det;
		real lam2 = (A11 * target2 - A12 * target1) / det;

		rb1->applyDeltaPos(rb1->mInv() * (xhat * lam1 + yhat * lam2), rb1->IInv() * (d1crossx * lam1 + d1crossy * lam2));
		rb2->applyDeltaPos(-rb2->mInv() * (xhat * lam1 + yhat * lam2), -rb2->IInv() * (d2crossx * lam1 + d2crossy * lam2));
	}
}

void PinConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(rb1->mInv() * (xhat * accLam1 + yhat * accLam2), rb1->IInv() * (d1crossx * accLam1 + d1crossy * accLam2));
		rb2->applyDeltaVel(-rb2->mInv() * (xhat * accLam1 + yhat * accLam2), -rb2->IInv() * (d2crossx * accLam1 + d2crossy * accLam2));
	}
	else
	{
		accLam1 = accLam2 = 0;
	}
}

void PinConstraint::prepareVelSolver()
{
	updateCachedData();
}

void PinConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	vec2 d1 = globalPoint1 - rb1->position();
	vec2 d2 = globalPoint2 - rb2->position();

	vec2 disp = globalPoint1 - globalPoint2;
	C1 = disp.x;
	C2 = disp.y;

	d1crossx = zcross(d1, xhat);
	d1crossy = zcross(d1, yhat);
	d2crossx = zcross(d2, xhat);
	d2crossy = zcross(d2, yhat);

	gradC1[2] = d1crossx;
	gradC1[5] = -d2crossx;
	gradC2[2] = d1crossy;
	gradC2[5] = -d2crossy;

	A11 = rb1->mInv() + rb1->IInv() * d1crossx * d1crossx + rb2->mInv() + rb2->IInv() * d2crossx * d2crossx;
	A22 = rb1->mInv() + rb1->IInv() * d1crossy * d1crossy + rb2->mInv() + rb2->IInv() * d2crossy * d2crossy;
	A12 = rb1->IInv() * d1crossx * d1crossy + rb2->IInv() * d2crossx * d2crossy;

	det = A11 * A22 - A12 * A12;
}
