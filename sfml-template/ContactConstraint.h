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
	ContactConstraint(const PhysicsSettings& ps, RigidBody* rb1 = 0, RigidBody* rb2 = 0);

	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void updateCache() = 0;

	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	virtual bool matches(const ContactConstraint* other) const = 0;
	virtual bool matches(const PolyPolyContact* other) const = 0;
	virtual bool matches(const CircleCircleContact* other) const = 0;

	virtual void rebuild() = 0;
	virtual void rebuildFrom(ContactConstraint* other) = 0;

	int numPersist = 0;

protected:
	void solvePointFriction(ContactPoint& cp);
	void solvePointVel(ContactPoint& cp);
	void solvePointPos(ContactPoint& cp);
	void warmStartPoint(ContactPoint& cp); 
	void updatePointCache(ContactPoint& cp);

	real mu = 0;
	real e = 0;

	vec2 n, t;

	RigidBody* rb1 = nullptr;
	RigidBody* rb2 = nullptr;

	const PhysicsSettings& ps;

private:

};

