#pragma once
#include "RigidBody.h"

class Circle : public RigidBody
{
public:
	Circle(const PhysicsSettings& ps, real rad, real mInv = 0);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override { return nullptr; }
	std::unique_ptr<ContactConstraint> checkCollision(Circle* other) override;

	bool pointInside(const vec2& p) const override;

	void onMove() override { }
	void onRotate() override { }

	real radius() const { return rad; }


private:
	void initShape();

	real rad = 1;

	sf::CircleShape shape;
};

