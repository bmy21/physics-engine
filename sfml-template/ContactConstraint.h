#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "PhysicsSettings.h"
#include "ContactPoint.h"

class PolyPolyContact;
class CircleCircleContact;
class PolyCircleContact;
class RigidBody;

class ContactConstraint
{
public:
	ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2);

	virtual void init() = 0;
	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void prepareVelSolver() = 0;
	virtual void getImpulsesFrom(ContactConstraint* other) = 0;

	void markForRemoval() { remove = true; }
	bool removeFlagSet() const { return remove; }

	virtual void draw(sf::RenderWindow& window, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;
	
	int numPersist = 0;

protected:
	const PhysicsSettings& ps;
	RigidBody* const rb1;
	RigidBody* const rb2;

private:

	bool remove = false;
};

