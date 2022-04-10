#include "ConvexPolygon.h"

ConvexPolygon::ConvexPolygon()
{
	

	shape.setPosition(100, 100);
	//shape.setRotation(theta);

	int npoints = 5;
	real sideLength = 50;

	makeRegularPolygon(npoints, sideLength);

	initShape();
}

void ConvexPolygon::update(real dt)
{

}

void ConvexPolygon::draw(sf::RenderWindow& window, real fraction)
{
	//shape.setPosition(sf::Vector2f(iPos));
	//shape.setRotation(iTheta * 180.0 / 3.14159);

	window.draw(shape);
}

void ConvexPolygon::makeRegularPolygon(int nsides, real sideLength)
{
	// Shouldn't be calling more than once per polygon!
	assert(points.empty());
		
	real theta = 2 * pi / nsides;
	real r = sideLength / (2 * std::sin(theta / 2));
	

	for (int i = 0; i < nsides; ++i)
	{
		real angle = i * theta;

		// Ensure bottom edge is horizontal
		// Bottom-right corner makes angle (90 - theta/2) deg with horizontal
		angle += pi / 2 - theta / 2;

		vec2 pos = { r * std::cos(angle), r * std::sin(angle) };

		points.push_back(pos);
	}
}

void ConvexPolygon::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(2);


	auto npoints = points.size();

	shape.setPointCount(npoints);

	for (decltype(npoints) i = 0; i < npoints; ++i)
	{
		shape.setPoint(i, points[i]);
	}
}
