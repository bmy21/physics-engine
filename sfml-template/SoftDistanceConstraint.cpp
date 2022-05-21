#include "SoftDistanceConstraint.h"

SoftDistanceConstraint::SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real d, real tOsc, real dampingRatio, real dtInv):
	rb(rb), fixedPoint(fixedPoint), localPoint(localPoint),
	d(d), dtInv(dtInv)
{
	k = 4 * pi * pi / (tOsc * tOsc * rb->mInv);
	b = dampingRatio * 2 * std::sqrt(k / rb->mInv);

	storedVel = rb->velocity();
	storedAngVel = rb->angVel();
}

void SoftDistanceConstraint::correctVel()
{
	vec2 p = transform(localPoint, rb->position(), rb->angle());
	vec2 dir1 = { 1,0 };// fixedPoint - p;
	real mag = magnitude(dir1);

	if (mag > 0)
	{
		dir1 /= mag;
	}

	vec2 dir2 = perp(dir1);

	real crossFactor1 = zcross(p - rb->position(), dir1);
	real crossFactor2 = zcross(p - rb->position(), dir2);

	real vDotGradC1 = dot(rb->velocity(), dir1) + rb->angVel() * crossFactor1;
	real vDotGradC2 = dot(rb->velocity(), dir2) + rb->angVel() * crossFactor2;

	real muInv1 = rb->mInv + rb->IInv * crossFactor1 * crossFactor1;
	real muInv2 = rb->mInv + rb->IInv * crossFactor2 * crossFactor2;

	real h = 1 / dtInv;

	real beta = k / (h * k + b);
	real gamma = 1 / (h * (h * k + b));

	real C1 = dot(p - fixedPoint, dir1);
	real C2 = dot(p - fixedPoint, dir2);

	real alpha1 = -vDotGradC1 - beta * C1 - gamma * accLam1;
	real alpha2 = -vDotGradC2 - beta * C2 - gamma * accLam2;


	real A11 = muInv1 + gamma;
	real A22 = muInv2 + gamma;
	real A12 = rb->IInv * crossFactor1 * crossFactor2;
	real det = A11 * A22 - A12 * A12;

	if (det != 0)
	{
		real lam1 = (A22 * alpha1 - A12 * alpha2) / det;
		real lam2 = (A11 * alpha2 - A12 * alpha1) / det;

		real prevAccLam1 = accLam1;
		real prevAccLam2 = accLam2;

		accLam1 += lam1;
		accLam2 += lam2;


		// TODO: fMax should depend on the mass? i.e. limit acceleration?
		real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) * dtInv;

		real fMax = 500;

		std::cout << force << "\n";

		if (force > fMax)
		{
			//std::cout << force << '\n';
			accLam1 *= fMax / force;
			accLam2 *= fMax / force;
		}

		lam1 = accLam1 - prevAccLam1;
		lam2 = accLam2 - prevAccLam2;

		rb->applyDeltaVel(dir1 * rb->mInv * lam1, crossFactor1 * rb->IInv * lam1);
		rb->applyDeltaVel(dir2 * rb->mInv * lam2, crossFactor2 * rb->IInv * lam2);



		//real E = 0.5 * k * C1 * C1 + (0.5 / rb->mInv) * rb->KE(); //- 0.5*k*h*rb->velocity().x*C1;
		//std::cout << E << "\n";

	}
}

void SoftDistanceConstraint::correctPos()
{
}

void SoftDistanceConstraint::warmStart()
{
	//accLam1 = accLam2 = 0;


	vec2 p = transform(localPoint, rb->position(), rb->angle());
	
	vec2 dir1 = { 1,0 }; // fixedPoint - p;
	real mag = magnitude(dir1);

	if (mag > 0)
	{
		dir1 /= mag;
	}

	vec2 dir2 = perp(dir1);


	real crossFactor1 = zcross(p - rb->position(), dir1);
	real crossFactor2 = zcross(p - rb->position(), dir2);

	rb->applyDeltaVel(dir1 * rb->mInv * accLam1, crossFactor1 * rb->IInv * accLam1);
	rb->applyDeltaVel(dir2 * rb->mInv * accLam2, crossFactor2 * rb->IInv * accLam2);
}
