#pragma once

#include "Utils.h"
#include "PhysicsSettings.h"

class RigidBody;

class Constraint
{
public:
	Constraint(const PhysicsSettings& ps, std::initializer_list<RigidBody*> rigidBodies);
	virtual ~Constraint();

	virtual void draw(sf::RenderWindow& window, real fraction) {  }

	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void prepareVelSolver() = 0;

	void markForRemoval() { remove = true; }
	bool removeFlagSet() const { return remove; }

protected:
	const PhysicsSettings& ps;

private:
	// Keep track of which rigid bodies this constraint acts on
	const std::unordered_set<RigidBody*> rigidBodies;

	bool remove = false;
};

