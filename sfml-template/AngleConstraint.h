#pragma once
#include "Constraint.h"
class AngleConstraint : public Constraint
{
public:
	AngleConstraint(RigidBody* rb1, RigidBody* rb2, real angleDiff, const PhysicsSettings& ps);

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;

private:
	void updateCachedData();

	RigidBody* rb1;
	RigidBody* rb2;
	
	real angleDiff = 0;
	real massFactor = 0;
	real accLam = 0;
};

