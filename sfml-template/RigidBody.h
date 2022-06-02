#pragma once

#include "Utils.h"
#include "ContactConstraint.h"
#include "PhysicsSettings.h"

class ConvexPolygon;
class ContactConstraint;

class RigidBody
{
public:
	RigidBody();

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, 
		bool debug = false, sf::Text* text = nullptr) = 0;

	virtual std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) = 0;

	virtual bool pointInside(const vec2& p) = 0;

	void moveTo(const vec2& p);
	void rotateTo(real t);

	// Functions called whenever position/orientation is changed
	virtual void onMove() = 0;
	virtual void onRotate() = 0;

	void integrateVel(real dt);
	void integratePos(real dt);

	vec2 interpolatePos(real fraction) const;
	real interpolateAngle(real fraction) const;

	real angle() const { return theta; }
	vec2 position() const { return pos; }
	real angVel() const { return omega; }
	vec2 velocity() const { return vel; }
	real KE() const { return 0.5 * dot(vel, vel) / mInv; }

	void applyDeltaVel(const vec2& dv, real dw);
	void applyDeltaPos(const vec2& dr, real dth);

	void applyDamping(real dt);

	vec2 pointVel(const vec2& p) const { return vel + omega * -perp(p - pos); }


	// TODO: make these private
	real mInv = 1, IInv = 1;
	real grav = 0;

	real linearDamp = 0, angularDamp = 0;

private:
	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	vec2 posPrev;
	real thetaPrev = 0;
};