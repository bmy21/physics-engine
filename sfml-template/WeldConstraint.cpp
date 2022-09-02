#include "WeldConstraint.h"
#include "RigidBody.h"

WeldConstraint::WeldConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, real refAngle,
	const PhysicsSettings& ps, bool relativeToRefPoints):
	rb1(rb1), rb2(rb2), localPoint1(localPoint1), localPoint2(localPoint2), refAngle(refAngle),
	xhat({ 1, 0 }), yhat({ 0, 1 }),
	Constraint(ps, { rb1, rb2 })
{
	if (relativeToRefPoints)
	{
		this->localPoint1 += rb1->getRefPoint();
		this->localPoint2 += rb2->getRefPoint();
	}
}

void WeldConstraint::correctVel()
{
	real vDotGradC1 = rb1->velocity().x + rb1->angVel() * d1crossx - rb2->velocity().x - rb2->angVel() * d2crossx;
	real vDotGradC2 = rb1->velocity().y + rb1->angVel() * d1crossy - rb2->velocity().y - rb2->angVel() * d2crossy;
	real vDotGradC3 = rb1->angVel() - rb2->angVel();

	if (det != 0)
	{
		real target1 = -vDotGradC1, target2 = -vDotGradC2, target3 = -vDotGradC3;

		real lam1 = (aAdj * target1 + bAdj * target2 + cAdj * target3) / det;
		real lam2 = (dAdj * target1 + eAdj * target2 + fAdj * target3) / det;
		real lam3 = (gAdj * target1 + hAdj * target2 + iAdj * target3) / det;

		accLam1 += lam1;
		accLam2 += lam2;
		accLam3 += lam3;

		rb1->applyDeltaVel(rb1->mInv() * (xhat * lam1 + yhat * lam2), rb1->IInv() * (d1crossx * lam1 + d1crossy * lam2 + lam3));
		rb2->applyDeltaVel(-rb2->mInv() * (xhat * lam1 + yhat * lam2), -rb2->IInv() * (d2crossx * lam1 + d2crossy * lam2 + lam3));
	}
}

void WeldConstraint::correctPos()
{
	updateCachedData();

	if (det != 0)
	{
		real target1 = -ps.beta * C1, target2 = -ps.beta * C2, target3 = -ps.beta * C3;
		real lam1 = (aAdj * target1 + bAdj * target2 + cAdj * target3) / det;
		real lam2 = (dAdj * target1 + eAdj * target2 + fAdj * target3) / det;
		real lam3 = (gAdj * target1 + hAdj * target2 + iAdj * target3) / det;

		rb1->applyDeltaPos(rb1->mInv() * (xhat * lam1 + yhat * lam2), rb1->IInv() * (d1crossx * lam1 + d1crossy * lam2 + lam3));
		rb2->applyDeltaPos(-rb2->mInv() * (xhat * lam1 + yhat * lam2), -rb2->IInv() * (d2crossx * lam1 + d2crossy * lam2 + lam3));
	}
}

void WeldConstraint::warmStart()
{
	if (ps.warmStart)
	{
		rb1->applyDeltaVel(rb1->mInv() * (xhat * accLam1 + yhat * accLam2), rb1->IInv() * (d1crossx * accLam1 + d1crossy * accLam2 + accLam3));
		rb2->applyDeltaVel(-rb2->mInv() * (xhat * accLam1 + yhat * accLam2), -rb2->IInv() * (d2crossx * accLam1 + d2crossy * accLam2 + accLam3));
	}
	else
	{
		accLam1 = accLam2 = accLam3 = 0;
	}
}

void WeldConstraint::prepareVelSolver()
{
	updateCachedData();
}

void WeldConstraint::updateCachedData()
{
	globalPoint1 = rb1->pointToGlobal(localPoint1);
	globalPoint2 = rb2->pointToGlobal(localPoint2);

	vec2 d1 = globalPoint1 - rb1->position();
	vec2 d2 = globalPoint2 - rb2->position();

	vec2 disp = globalPoint1 - globalPoint2;
	C1 = disp.x;
	C2 = disp.y;
	C3 = wrap(rb1->angle() - rb2->angle() - refAngle);

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
	A33 = rb1->IInv() + rb2->IInv();

	A12 = rb1->IInv() * d1crossx * d1crossy + rb2->IInv() * d2crossx * d2crossy;
	A13 = rb1->mInv() * d1crossx + rb2->mInv() * d2crossx;
	A23 = rb1->mInv() * d1crossy + rb2->mInv() * d2crossy;

	aAdj = A22 * A33 - A23 * A23;
	bAdj = A13 * A23 - A12 * A22;
	cAdj = A12 * A23 - A13 * A22;
	
	dAdj = A23 * A13 - A12 * A33;
	eAdj = A11 * A33 - A13 * A13;
	fAdj = A13 * A12 - A11 * A23;

	gAdj = A12 * A23 - A22 * A13;
	hAdj = A12 * A13 - A11 * A23;
	iAdj = A11 * A22 - A12 * A12;

	det = A11 * (A22 * A33 - A23 * A23) - A12 * (A12 * A33 - A23 * A13) + A13 * (A12 * A23 - A13 * A22);
}
