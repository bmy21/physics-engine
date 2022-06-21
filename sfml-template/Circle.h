#pragma once
#include "RigidBody.h"
#include "CircleCircleContact.h"
#include "ConvexPolygon.h"

class ConvexPolygon;

class Circle : public RigidBody
{
public:
	Circle(const PhysicsSettings& ps, real rad, real mInv = 0);

	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override; // Need checkCirclePolyCollide(this, other)?
	std::unique_ptr<ContactConstraint> checkCollision(Circle* other) override;

	bool pointInside(const vec2& p) const override;
	void updateAABB() override;
	void updateFatAABB(real w) override;

	void onMove() override { }
	void onRotate() override { }

	real radius() const { return rad; }
	vec2 furthestPoint(const vec2& d);

private:
	void initShape();

	real rad = 1;

	sf::CircleShape shape;
};

