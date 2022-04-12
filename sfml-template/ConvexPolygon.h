#pragma once

#include "RigidBody.h"

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon(int npoints, real sideLength);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

private:

	void createRegularPolygon(real sideLength);
	void initShape();

	const int npoints;
	std::vector<vec2> points;
	sf::ConvexShape shape;
};

