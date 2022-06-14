#pragma once

#include "Utils.h"
#include "ContactConstraint.h"
#include "PhysicsSettings.h"
#include "AABB.h"

class ConvexPolygon;
class Circle;
class ContactConstraint;

using idType = unsigned long;

class RigidBody
{
public:
	RigidBody(const PhysicsSettings& ps, real mInv = 0, real IInv = 0);

	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, 
		bool debug = false, sf::Text* text = nullptr) = 0;

	virtual std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(Circle* other) = 0;

	virtual bool pointInside(const vec2& p) const = 0;
	virtual void updateAABB() = 0;

	vec2 pointToLocal(const vec2& p) const;
	vec2 pointToGlobal(const vec2& p) const;
	vec2 vecToLocal(const vec2& v) const;
	vec2 vecToGlobal(const vec2& v) const;

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
	real prevAngle() const { return thetaPrev; }
	vec2 position() const { return pos; }
	vec2 prevPosition() const { return posPrev; }

	real angVel() const { return omega; }
	vec2 velocity() const { return vel; }
	vec2 pointVel(const vec2& p) const { return vel + omega * -perp(p - pos); }

	real left() const { return aabb.left; }
	real bottom() const { return aabb.bottom; }
	real right() const { return aabb.right; }
	real top() const { return aabb.top; }

	real KE() const { return 0.5 * dot(vel, vel) / mInv; }

	void applyDeltaVel(const vec2& dv, real dw);
	void applyDeltaPos(const vec2& dr, real dth, bool update = true);

	void applyDamping(real dt);


	const idType id;

	// TODO: make these private
	real mInv = 0, IInv = 0;

protected:
	const PhysicsSettings& ps;
	AABB aabb;

private:
	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	vec2 posPrev;
	real thetaPrev = 0;

	static inline idType counter = 0;
};