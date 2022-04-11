#include "ConvexPolygon.h"


ConvexPolygon::ConvexPolygon(int npoints, real sideLength):
	npoints(npoints)
{
	moveTo({ 1, 7 });
	rotateTo(10.0 * pi / 180);


	//omega = 40.0 * pi / 180;
	//vel = { 7, -7 };

	createRegularPolygon(sideLength);
	initShape();
}

void ConvexPolygon::update(real dt)
{

}

void ConvexPolygon::draw(sf::RenderWindow& window, real pixPerUnit, real fraction)
{
	vec2 ipos = interpolatePos(fraction);

	for (int i = 0; i < npoints; ++i)
	{
		shape.setPoint(i, (ipos + rotate(points[i], theta)) * pixPerUnit);
	}

	window.draw(shape);
}

void ConvexPolygon::createRegularPolygon(real sideLength)
{
	// Shouldn't be calling more than once per polygon!
	assert(points.empty());
		
	real theta = 2 * pi / npoints;
	real r = sideLength / (2 * std::sin(theta / 2));
	

	for (int i = 0; i < npoints; ++i)
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

	shape.setPointCount(npoints);
}
