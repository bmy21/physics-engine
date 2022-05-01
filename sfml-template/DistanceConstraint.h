#pragma once
#include "Constraint.h"

class DistanceConstraint : public Constraint
{
public:
	DistanceConstraint();

	void correctVel() override;
	void correctPos() override;
	void warmStart() override;

	vec2 point;
	RigidBody* rb;
	

private:
	
	real storedLambda = 0;

};
