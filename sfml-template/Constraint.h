#pragma once

#include "Utils.h"
#include "RigidBody.h"


class Constraint
{
public:
	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;

	void setBeta(real b) { posBeta = b; }
	real beta() const { return posBeta; }

private:
	real posBeta = 0.3;
};
