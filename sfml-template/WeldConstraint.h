#pragma once
#include "Constraint.h"


// Weld constraint gets its own class as it requires simultaneous solution of 3 constraints

class WeldConstraint : public Constraint
{
public:
	WeldConstraint(RigidBody* rb1, RigidBody* rb2, const vec2& localPoint1, const vec2& localPoint2, real refAngle,
		const PhysicsSettings& ps, bool relativeToRefPoints = false);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;

private:
	void updateCachedData();

	RigidBody* rb1;
	RigidBody* rb2;

	real C1 = 0, C2 = 0, C3 = 0;
	std::array<real, 6> gradC1{ 1, 0, 0, -1, 0, 0 };
	std::array<real, 6> gradC2{ 0, 1, 0, 0, -1, 0 };
	std::array<real, 6> gradC3{ 0, 0, 1, 0, 0, -1 };

	real A11 = 0, A22 = 0, A33 = 0, A12 = 0, A13 = 0, A23 = 0;
	real aAdj = 0, bAdj = 0, cAdj = 0, dAdj = 0, eAdj = 0, fAdj = 0, gAdj = 0, hAdj = 0, iAdj = 0;
	real det = 0;

	vec2 localPoint1, localPoint2;
	vec2 globalPoint1, globalPoint2;

	real refAngle = 0;
	real d1crossx = 0, d2crossx = 0;
	real d1crossy = 0, d2crossy = 0;

	real accLam1 = 0, accLam2 = 0, accLam3 = 0;

	const vec2 xhat, yhat;
};