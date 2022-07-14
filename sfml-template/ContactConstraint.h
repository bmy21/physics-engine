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

	void init();
	void correctVel();
	void correctPos();
	void warmStart();
	void prepareVelSolver();
	void getImpulsesFrom(ContactConstraint* other);

	void markForRemoval() { remove = true; }
	bool removeFlagSet() const { return remove; }

	void draw(sf::RenderWindow& window, real fraction, bool debug = false, sf::Text* text = nullptr);
	
	int numPersist = 0;

protected:
	// The details of the below functions depend on the specific types of rigid body involved
	virtual void initPoints() = 0;
	virtual void updateNormal() = 0;
	virtual void rebuildPoint(ContactPoint& cp) = 0;

	void enableRollingFriction() { rollingFriction = true; }
	void disableRollingFriction() { rollingFriction = false; }
	bool idsMatch(const ContactConstraint* other) const;

	std::vector<ContactPoint> contactPoints;
	int ncp = 0;

	// Collision normal & tangent (shared by all contact points)
	vec2 n, t;

	const PhysicsSettings& ps;

private:
	void storeTargetVelocities();
	void updateTangent();
	void prepareSimulSolver();

	bool simulSolveVel();
	void simulSolvePos();

	void updateNormalFactors(ContactPoint& cp);
	void updateTangentFactors(ContactPoint& cp);

	void solvePointRollFriction(ContactPoint& cp);
	void solvePointFriction(ContactPoint& cp);
	void solvePointVel(ContactPoint& cp);
	void solvePointPos(ContactPoint& cp);
	void warmStartPoint(ContactPoint& cp);

	RigidBody* const rb1;
	RigidBody* const rb2;

	real mu = 0;
	real e = 0;

	bool remove = false;

	bool rollingFriction = false;
	real rfLength = 0;

	// Cached data for simultaneous solution
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;
	real A12 = 0, det = 0, norm = 0;
};

