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
	void rebuildFrom(ContactConstraint* other);

	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	virtual bool matches(const ContactConstraint* other) const = 0;
	virtual bool matches(const PolyPolyContact* other) const = 0;
	virtual bool matches(const CircleCircleContact* other) const = 0;
	virtual bool matches(const PolyCircleContact* other) const = 0;
	
	int numPersist = 0;

protected:
	// The details of the below functions depend on the specific types of rigid body involved,
	// so they need to be virtual
	virtual void initPoints() = 0;
	virtual void updateNormal() = 0;
	virtual void rebuildPoint(ContactPoint& cp) = 0;
	virtual void onRebuildFrom(ContactConstraint* other) = 0;

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

	void solvePointFriction(ContactPoint& cp);
	void solvePointVel(ContactPoint& cp);
	void solvePointPos(ContactPoint& cp);
	void warmStartPoint(ContactPoint& cp);

	RigidBody* rb1 = nullptr;
	RigidBody* rb2 = nullptr;

	real mu = 0;
	real e = 0;

	// Cached data for simultaneous solution
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;
	real A12 = 0, det = 0, norm = 0;
};

