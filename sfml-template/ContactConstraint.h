#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "PhysicsSettings.h"
#include "ContactPoint.h"

class PolyPolyContact;
class CircleCircleContact;
class RigidBody;

class ContactConstraint
{
public:
	ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2);

	void init();

	void correctVel();
	void correctPos();
	void warmStart();
	void updateCache();

	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	virtual bool matches(const ContactConstraint* other) const = 0;
	virtual bool matches(const PolyPolyContact* other) const = 0;
	virtual bool matches(const CircleCircleContact* other) const = 0;
	virtual void rebuildFrom(ContactConstraint* other) = 0;

	int numPersist = 0;

protected:
	

	virtual void initPoints() = 0;
	virtual void rebuildPoints() = 0;
	virtual void updateNormal() = 0;

	real mu = 0;
	real e = 0;

	RigidBody* rb1 = nullptr;
	RigidBody* rb2 = nullptr;

	std::vector<ContactPoint> contactPoints;

	// TODO: make private
	int ncp = -1;

	// Collision normal & tangent (shared by all contact points)
	vec2 n, t;

	const PhysicsSettings& ps;

private:
	void storeRelativeVelocities();
	void solvePointFriction(ContactPoint& cp);
	void solvePointVel(ContactPoint& cp);
	void solvePointPos(ContactPoint& cp);
	void warmStartPoint(ContactPoint& cp);
	void updatePointCache(ContactPoint& cp);

	// Cached data for simultaneous solution
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;
	real A12 = 0, det = 0, norm = 0;
};

