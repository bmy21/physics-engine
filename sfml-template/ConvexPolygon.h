#pragma once

#include "RigidBody.h"

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon();
	ConvexPolygon(int nsides, real sideLength, real pixPerUnit);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction) override;

private:

	void createRegularPolygon(int nsides, real sideLength);
	void initShape(real pixPerUnit = 1);

	std::vector<vec2> points;
	sf::ConvexShape shape;

};

