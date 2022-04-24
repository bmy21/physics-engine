#pragma once
#include "Constraint.h"

class DistanceConstraint : public Constraint
{
public:
	void correctVel() override;
	void correctPos() override;

	vec2 point;
	RigidBody* rb;

private:
	
	

};

