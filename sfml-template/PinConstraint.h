#pragma once
#include "Constraint.h"


// Pin constraint gets its own class as it requires a simultaneous solution

class PinConstraint : public Constraint
{
public:
	PinConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2,
		const PhysicsSettings& ps, bool relativeToRefPoints = false);
	
	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;
	
private:
	void updateCachedData();

	RigidBody* rb1;
	RigidBody* rb2;
	
	real C1 = 0, C2 = 0;
	std::array<real, 6> gradC1{ 1, 0, 0, -1, 0, 0 };
	std::array<real, 6> gradC2{ 0, 1, 0, 0, -1, 0 };

	real A11 = 0, A22 = 0, A12 = 0;
	real det = 0;

	vec2 localPoint1, localPoint2;
	vec2 globalPoint1, globalPoint2;

	real d1crossx = 0, d2crossx = 0;
	real d1crossy = 0, d2crossy = 0;

	real accLam1 = 0, accLam2 = 0;

	const vec2 xhat, yhat;
};

