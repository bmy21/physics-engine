#pragma once

#include "Utils.h"
#include "RigidBody.h"


class Constraint
{
public:
	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void updateCache() = 0;

	void markForRemoval() { remove = true; }
	bool removeFlagSet() const { return remove; }

	void setBeta(real b) { posBeta = b; }
	real beta() const { return posBeta; }

private:
	real posBeta = 0.3;

	bool remove = false;
};

