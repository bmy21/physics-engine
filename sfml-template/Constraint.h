#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "PhysicsSettings.h"


class Constraint
{
public:
	Constraint(const PhysicsSettings& ps);

	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void updateCache() = 0;

	void markForRemoval() { remove = true; }
	bool removeFlagSet() const { return remove; }

protected:
	const PhysicsSettings& ps;

private:
	bool remove = false;
};

