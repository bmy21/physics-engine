#include "SoftDistanceConstraint.h"

SoftDistanceConstraint::SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real d, real tOsc, real tDec, real dtInv):
	rb(rb), fixedPoint(fixedPoint), localPoint(localPoint),
	d(d), dtInv(dtInv)
{
	//b = 2 * std::log(2.f) / (tDec * rb->mInv);

	k = 4 * pi * pi / (tOsc * tOsc * rb->mInv);
	b = 0;

	// critical damping
	//b = 2 * std::sqrt(k * rb->mInv);


	storedVel = rb->velocity();
	storedAngVel = rb->angVel();
}

void SoftDistanceConstraint::correctVel()
{

	if (0)
	{
		//std::cout << accLam1 << "\t\t" << accLam2 << "\n";

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
		//std::cout << h << "\n";

		real beta = k / (h * k + b);
		real gamma = 1 / (h*(h * k + b)); // TODO: why no /h?

		real C1 = dot(p - fixedPoint, dir1);
		real C2 = dot(p - fixedPoint, dir2);

		// TODO: Why accLam *h?

		real alpha1 = -vDotGradC1 - beta * C1 - gamma * accLam1;
		real alpha2 = -vDotGradC2 - beta * C2 - gamma * accLam2;


		real A12 = rb->IInv * crossFactor1 * crossFactor2;
		real det = muInv1 * muInv2 - A12 * A12;

		// std::cout << det << "\n";

		if (det != 0)
		{
			real lam1 = (muInv2 * alpha1 - A12 * alpha2) / det;
			real lam2 = (muInv1 * alpha2 - A12 * alpha1) / det;

			//std::cout << lam1 << "\n" << lam2 << "\n";
			
			real prevAccLam1 = accLam1, prevAccLam2 = accLam2;

			accLam1 += lam1;
			accLam2 += lam2;
		
			// TODO: fMax should depend on the mass? i.e. limit acceleration?
			real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) * dtInv;

			real fMax = 300;

			std::cout << force << "\n";

			if (force > fMax)
			{
				//std::cout << force << '\n';
				//accLam1 *= fMax / force;
				//accLam2 *= fMax / force;
			}

			lam1 = accLam1 - prevAccLam1;
			lam2 = accLam2 - prevAccLam2;


			//lam1 = lam1 - accLam1;
			//lam2 = lam2 - accLam2;

			rb->applyDeltaVel(dir1 * rb->mInv * lam1, crossFactor1 * rb->IInv * lam1);
			rb->applyDeltaVel(dir2 * rb->mInv * lam2, crossFactor2 * rb->IInv * lam2);
			

			return;
		}

		std::cout << "!!! ";
		
		return;
	}



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

		real muInv1 = rb->mInv + rb->IInv * crossFactor1 * crossFactor1;
		real muInv2 = rb->mInv + rb->IInv * crossFactor2 * crossFactor2;

		real h = 1 / dtInv;
		//std::cout << h << "\n";

		real beta = k / (h * k + b);
		real gamma = 1 / (h * (h * k + b));

		real C1 = dot(p - fixedPoint, dir1);
		real C2 = dot(p - fixedPoint, dir2);

		// TODO: Why accLam *h?
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

			//std::cout << lam1 << "\n" << lam2 << "\n";

			//real prevAccLam1 = accLam1, prevAccLam2 = accLam2;

			accLam1 += lam1;
			accLam2 += lam2;

			//std::cout

			// TODO: fMax should depend on the mass? i.e. limit acceleration?
			real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) * dtInv;

			real fMax = 300;

			//std::cout << force << "\n";

			if (force > fMax)
			{
				//std::cout << force << '\n';
				//accLam1 *= fMax / force;
				//accLam2 *= fMax / force;
			}

			//lam1 = accLam1 - prevAccLam1;
			//lam2 = accLam2 - prevAccLam2;


			//lam1 = lam1 - accLam1;
			//lam2 = lam2 - accLam2;

			rb->applyDeltaVel(dir1 * rb->mInv * lam1, crossFactor1 * rb->IInv * lam1);
			rb->applyDeltaVel(dir2 * rb->mInv * lam2, crossFactor2 * rb->IInv * lam2);



			//real E = 0.5 * k * C1 * C1 + (0.5 / rb->mInv) * std::pow(magnitude(rb->velocity()), 2) - 0.5*k*h*rb->velocity().x*C1;
			//std::cout << E << "\n";

			return;
		}

		std::cout << "!!! ";

		return;
	}

	


	// TODO: proper clamping - restrict effective length of spring by projecting point?
	vec2 p = transform(localPoint, rb->position(), rb->angle());

	vec2 dir = vec2(1, 0); // -dot(p - fixedPoint, { 1,0 }) * vec2(1, 0);
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
		dLambda = -(b * vDotGradC + k * (C - d)) / (dtInv + b * massFactor)     - accLam1;
		//dLambda = std::clamp(dLambda, -.1f, .1f);
	}

	accLam1 += dLambda;

	//std::cout << vDotGradC << " --- " << b << " --- " << b * vDotGradC + k * (C - d) << " --- " << dLambda << '\n';

	//std::cout << accLam1 << "\n";

	rb->applyDeltaVel(dir * rb->mInv * dLambda, crossFactor * rb->IInv * dLambda);


	real E = 0.5 * k * C * C + (0.5 / rb->mInv) * std::pow(magnitude(rb->velocity()), 2);
	std::cout << E << "\n";
	return;
	
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
		dLambda = -(b * vDotGradC + k * (C - d)) / (dtInv + b * massFactor)      - accLam2;
		//dLambda = std::clamp(dLambda, -.1f, .1f);
	}
	
	accLam2 += dLambda;
	rb->applyDeltaVel(dir * rb->mInv * dLambda, crossFactor * rb->IInv * dLambda);


	//real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) * dtInv;
	//std::cout << force << "\n";
	
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
