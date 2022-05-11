#include "SoftDistanceConstraint.h"

SoftDistanceConstraint::SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real d, real k, real b, real dtInv):
	rb(rb), fixedPoint(fixedPoint), localPoint(localPoint),
	d(d), k(k), b(b), dtInv(dtInv)
{
	storedVel = rb->velocity();
	storedAngVel = rb->angVel();
}

void SoftDistanceConstraint::correctVel()
{
	if (1)
	{
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

		real vDotGradC1 = dot(rb->velocity(), dir1) + rb->angVel() * crossFactor1;
		real vDotGradC2 = dot(rb->velocity(), dir2) + rb->angVel() * crossFactor2;

		//real vDotGradC1 = dot(storedVel, dir1) + storedAngVel * crossFactor1;
		//real vDotGradC2 = dot(storedVel, dir2) + storedAngVel * crossFactor2;


		real muInv1 = rb->mInv + rb->IInv * crossFactor1 * crossFactor1;
		real muInv2 = rb->mInv + rb->IInv * crossFactor2 * crossFactor2;

		real h = 1 / dtInv;

		real beta = k / (h * k + b);
		real gamma = dtInv / (h * k + b);

		real C1 = dot(p - fixedPoint, dir1);
		real C2 = dot(p - fixedPoint, dir2);

		// TODO: Why accLam *h?


		real alpha1 = -vDotGradC1 - beta * C1 - gamma * accLam1*h;
		real alpha2 = -vDotGradC2 - beta * C2 - gamma * accLam2*h;


		real A12 = rb->mInv * 0 + rb->IInv * crossFactor1 * crossFactor2;
		real det = muInv1 * muInv2 - A12 * A12;


		if (det != 0)
		{
			real lam1 = (muInv2 * alpha1 - A12 * alpha2) / det;
			real lam2 = (muInv1 * alpha2 - A12 * alpha1) / det;

			//std::cout << lam1 << "\n" << lam2 << "\n";
			
			real prevAccLam1 = accLam1, prevAccLam2 = accLam2;

			accLam1 += lam1;
			accLam2 += lam2;
		
			real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) * dtInv;

			real fMax = 200;

			//std::cout << force << "\n";

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
			

			return;
		}

		std::cout << "!!! ";
		
		return;
	}

	


	// TODO: proper clamping - restrict effective length of spring by projecting point?
	vec2 p = transform(localPoint, rb->position(), rb->angle());

	vec2 dir = fixedPoint - p;// vec2(1, 0); // -dot(p - fixedPoint, { 1,0 }) * vec2(1, 0);
	real mag = magnitude(dir);

	if (mag > 0)
	{
		dir /= mag;
	}
	
	real crossFactor = zcross(p - rb->position(), dir);


	real vDotGradC = dot(rb->velocity(), dir) + rb->angVel() * crossFactor;
	real massFactor = rb->mInv + rb->IInv * crossFactor * crossFactor;

	real C = dot(p - fixedPoint, dir);

	real dLambda = 0;
	if (dtInv + b * massFactor != 0)
	{
		dLambda = -(b * vDotGradC + k * (C - d)) / (dtInv + b * massFactor);
		//dLambda = std::clamp(dLambda, -.1f, .1f);
	}

	rb->applyDeltaVel(dir * rb->mInv * dLambda, crossFactor * rb->IInv * dLambda);


	
	p = transform(localPoint, rb->position(), rb->angle());

	dir = perp(dir);//vec2(0, 1);
	mag = magnitude(dir);

	if (mag > 0)
	{
		dir /= mag;
	}

	crossFactor = zcross(p - rb->position(), dir);

	vDotGradC = dot(rb->velocity(), dir) + rb->angVel() * crossFactor;
	massFactor = rb->mInv + rb->IInv * crossFactor * crossFactor;

	C = dot(p - fixedPoint, dir);

	dLambda = 0;
	if (dtInv + b * massFactor != 0)
	{
		dLambda = -(b * vDotGradC + k * (C - d)) / (dtInv + b * massFactor);
		//dLambda = std::clamp(dLambda, -.1f, .1f);
	}

	rb->applyDeltaVel(dir * rb->mInv * dLambda, crossFactor * rb->IInv * dLambda);
}

void SoftDistanceConstraint::correctPos()
{
}

void SoftDistanceConstraint::warmStart()
{
	//accLam1 = accLam2 = 0;


	
	vec2 dir1 = { 1,0 }; // fixedPoint - p;
	real mag = magnitude(dir1);

	if (mag > 0)
	{
		dir1 /= mag;
	}

	vec2 dir2 = perp(dir1);

	vec2 p = transform(localPoint, rb->position(), rb->angle());

	real crossFactor1 = zcross(p - rb->position(), dir1);
	real crossFactor2 = zcross(p - rb->position(), dir2);

	rb->applyDeltaVel(dir1 * rb->mInv * accLam1, crossFactor1 * rb->IInv * accLam1);
	rb->applyDeltaVel(dir2 * rb->mInv * accLam2, crossFactor2 * rb->IInv * accLam2);
}
