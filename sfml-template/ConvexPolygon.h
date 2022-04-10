#pragma once

#include "RigidBody.h"

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon();

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real fraction) override;

private:

	void makeRegularPolygon(int nsides);

	std::vector<vec2> points;
	sf::ConvexShape shape;

};

