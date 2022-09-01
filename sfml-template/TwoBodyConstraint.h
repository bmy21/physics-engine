#pragma once
#include "Constraint.h"

class TwoBodyConstraint : public Constraint
{
public:
	TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, const PhysicsSettings& ps);
	TwoBodyConstraint(RigidBody* rb1, RigidBody* rb2, real tOsc, real dampingRatio, const PhysicsSettings& ps);

	void makeSpringy(real tOsc, real dampingRatio);
	void makeRigid();
	
	void setAsDamper(real tDamp);
	void enableMotor(real vTarget, real fMax);
	void disableMotor();

	void setTarget(real t);
	void setRange(real small, real large);
	void allowFractionalChange(real frac);
	void removeLimits();

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;

protected:
	RigidBody* rb1;
	RigidBody* rb2;

	real C = 0;
	real C0 = 0, Cmin = 0, Cmax = 0;
	std::array<real, 6> gradC{};
	real massFactor = 0;

private:
	virtual void updateCachedData() = 0;

	void applyImpulse(real impulse);
	real getvDotGradC() const;

	bool isRigid = true;
	bool isLimited = false;
	bool motorEnabled = false;

	real beta = 0, gamma = 0;
	real accLam = 0;

	real accLower = 0, accUpper = 0, accMotor = 0;
	real motorTarget = 0;
	real motorMaxForce = 0;
};

