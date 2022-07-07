#pragma once
#include "RigidBody.h"
#include "CircleCircleContact.h"
#include "ConvexPolygon.h"

class ConvexPolygon;

class Circle : public RigidBody
{
public:
	Circle(const PhysicsSettings& ps, real rad, real mInv = 0);

	void draw(sf::RenderWindow& window, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override;
	std::unique_ptr<ContactConstraint> checkCollision(Circle* other) override;

	bool pointInside(const vec2& p) const override;
	void updateAABB() override;
	void updateFatAABB() override;

	// No data to update on move
	void onMove() override { }

	real radius() const { return rad; }
	vec2 furthestPoint(const vec2& d) const;

private:
	void initShape();

	real rad = 1;

	sf::CircleShape shape;

	real lineThicknessPix = 1;
	sf::RectangleShape line;
};

