#pragma once

#include "Utils.h"
#include "ContactConstraint.h"
#include "Constraint.h"
#include "PhysicsSettings.h"
#include "AABB.h"

class ConvexPolygon;
class Circle;
class ContactConstraint;
class AABBTree;

using idType = unsigned long;
using idPair = std::pair<idType, idType>;

struct idPairHasher
{
	size_t operator() (const idPair& p) const
	{
		idType x = p.first, y = p.second;
		return x < y ? y * y + x : x * x + x + y;
	}
};

class RigidBody
{
public:
	RigidBody(const PhysicsSettings& ps, real mInv = 0, real IInv = 0);

	virtual void draw(sf::RenderWindow& window, real fraction, 
		bool debug = false, sf::Text* text = nullptr) = 0;

	virtual std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) = 0;
	virtual std::unique_ptr<ContactConstraint> checkCollision(Circle* other) = 0;

	virtual bool pointInside(const vec2& p) const = 0;
	virtual void updateAABB() = 0;
	virtual void updateFatAABB() = 0;

	vec2 pointToLocal(const vec2& p) const;
	vec2 pointToGlobal(const vec2& p) const;
	vec2 vecToLocal(const vec2& v) const;
	vec2 vecToGlobal(const vec2& v) const;

	void moveTo(const vec2& p);
	void rotateTo(real t);

	// Function called whenever position/orientation is changed
	virtual void onMove() = 0;

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

	real left() const { return aabb.lower.x; }
	real bottom() const { return aabb.upper.y; }
	real right() const { return aabb.upper.x; }
	real top() const { return aabb.lower.y; }
	
	real KE() const { return 0.5 * dot(vel, vel) / mInv; }

	void applyDeltaVel(const vec2& dv, real dw);
	void applyDeltaPos(const vec2& dr, real dth, bool update = true);

	void applyDamping(real dt);

	AABB getAABB() const { return aabb; }
	AABB getFatAABB() const { return aabbFat; } 

	void markForRemoval();
	bool removeFlagSet() const { return remove; }
	void setAsRemovable() { removable = true; }
	void setAsUnremovable() { removable = false; }

	// NOTE: these should only be called by the Constraint class
	void addConstraint(Constraint* c) { constraints.insert(c); }
	void removeConstraint(Constraint* c) { constraints.erase(c); }
	
	const idType id;

	// TODO: make these private
	real mInv = 0, IInv = 0;

protected:
	const PhysicsSettings& ps;
	AABB aabb, aabbFat;

private:
	void markConstraintsForRemoval();

	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	vec2 posPrev;
	real thetaPrev = 0;

	bool removable = true;
	bool remove = false;

	// Keep track of which constraints are currently acting on this rigid body
	std::unordered_set<Constraint*> constraints;

	static inline idType counter = 0;
};