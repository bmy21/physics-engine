#pragma once

#include "Utils.h"
#include "ContactConstraint.h"

class ConvexPolygon;
class ContactConstraint;

class RigidBody
{
public:

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, 
		bool debug = false, sf::Text* text = nullptr) = 0;

	virtual std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) = 0;

	void moveTo(const vec2& p);
	void rotateTo(real t);

	// Functions called whenever position/orientation is changed
	virtual void onMove() = 0;
	virtual void onRotate() = 0;

	void integrateVel(real dt);
	void integratePos(real dt);

	vec2 interpolatePos(real fraction) const;
	real interpolateTheta(real fraction) const;

	real angle() const { return theta; }
	vec2 position() const { return pos; }


	// TODO: make these private
	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	real mInv = 0, IInv = 0;
	real grav = 0;

private:
	vec2 posPrev;
	real thetaPrev = 0;

};

