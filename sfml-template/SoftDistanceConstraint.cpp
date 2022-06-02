#include "SoftDistanceConstraint.h"

//SoftDistanceConstraint::SoftDistanceConstraint(RigidBody* rb, const vec2& fixedPoint, const vec2& localPoint, real dt, real tOsc, real dampingRatio, real fMax):
//	rb(rb), fixedPoint(fixedPoint), localPoint(localPoint), dt(dt), fMax(fMax)
//{
//	// Set k and b based on specified oscillation timescale and damping ratio
//	if (rb->mInv != 0)
//	{
//		k = 4 * pi * pi / (tOsc * tOsc * rb->mInv);
//		b = dampingRatio * 2 * std::sqrt(k / rb->mInv);
//
//		real denom = dt * k + b;
//		beta = k / denom;
//		gamma = 1 / (dt * denom);
//	}
//}
//
//void SoftDistanceConstraint::correctVel()
//{
//	real vDotGradC1 = dot(rb->velocity(), dir1) + rb->angVel() * crossFactor1;
//	real vDotGradC2 = dot(rb->velocity(), dir2) + rb->angVel() * crossFactor2;
//
//	real alpha1 = -vDotGradC1 - beta * C1 - gamma * accLam1;
//	real alpha2 = -vDotGradC2 - beta * C2 - gamma * accLam2;
//
//	if (det != 0)
//	{
//		real lam1 = (A22 * alpha1 - A12 * alpha2) / det;
//		real lam2 = (A11 * alpha2 - A12 * alpha1) / det;
//
//		real prevAccLam1 = accLam1;
//		real prevAccLam2 = accLam2;
//
//		accLam1 += lam1;
//		accLam2 += lam2;
//
//
//		// TODO: fMax should depend on the mass? i.e. limit acceleration?
//		real force = std::sqrt(accLam1 * accLam1 + accLam2 * accLam2) / dt;
//
//		//std::cout << force << " ---> " << force * rb->mInv << "\n";
//
//		if (force > fMax)
//		{
//			//std::cout << force << '\n';
//			accLam1 *= fMax / force;
//			accLam2 *= fMax / force;
//		}
//
//		lam1 = accLam1 - prevAccLam1;
//		lam2 = accLam2 - prevAccLam2;
//
//		rb->applyDeltaVel(dir1 * rb->mInv * lam1, crossFactor1 * rb->IInv * lam1);
//		rb->applyDeltaVel(dir2 * rb->mInv * lam2, crossFactor2 * rb->IInv * lam2);
//
//		//real E = 0.5 * k * C1 * C1 + (0.5 / rb->mInv) * rb->KE(); //- 0.5*k*h*rb->velocity().x*C1;
//		//std::cout << E << "\n";
//
//	}
//}
//
//void SoftDistanceConstraint::correctPos()
//{
//}
//
//void SoftDistanceConstraint::warmStart()
//{
//	//accLam1 = accLam2 = 0;
//
//	rb->applyDeltaVel(dir1 * rb->mInv * accLam1, crossFactor1 * rb->IInv * accLam1);
//	rb->applyDeltaVel(dir2 * rb->mInv * accLam2, crossFactor2 * rb->IInv * accLam2);
//}
//
//void SoftDistanceConstraint::updateCache()
//{
//	globalPoint = transform(localPoint, rb->position(), rb->angle());
//
//	dir1 = fixedPoint - globalPoint; // { 1,0 };
//
//	real mag = magnitude(dir1);
//
//	if (mag != 1 && mag != 0)
//	{
//		dir1 /= mag;
//	}
//
//	dir2 = perp(dir1);
//
//	crossFactor1 = zcross(globalPoint - rb->position(), dir1);
//	crossFactor2 = zcross(globalPoint - rb->position(), dir2);
//
//	muInv1 = rb->mInv + rb->IInv * crossFactor1 * crossFactor1;
//	muInv2 = rb->mInv + rb->IInv * crossFactor2 * crossFactor2;
//
//	C1 = dot(globalPoint - fixedPoint, dir1);
//	C2 = dot(globalPoint - fixedPoint, dir2);
//
//	A11 = muInv1 + gamma;
//	A22 = muInv2 + gamma;
//	A12 = rb->IInv * crossFactor1 * crossFactor2;
//	det = A11 * A22 - A12 * A12;
//}
