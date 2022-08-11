#pragma once
#include "Constraint.h"

class TwoBodyConstraint : public Constraint
{
public:
	TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, const PhysicsSettings& ps);
	TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, real tOsc, real dampingRatio, const PhysicsSettings& ps);

	void makeSpringy(real tOsc, real dampingRatio);
	void makeRigid();

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;

protected:
	RigidBody* rb1;
	RigidBody* rb2;

	real C = 0;
	std::array<real, 6> gradC{};
	real massFactor = 0;

private:
	virtual void updateCachedData() = 0;

	bool shouldCorrectPos = true;

	real beta = 0, gamma = 0;
	real accLam = 0;
};

