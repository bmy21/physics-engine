#include "ConvexPolygon.h"

ConvexPolygon::ConvexPolygon()
{
	shape.setFillColor(sf::Color::Black);// sf::Color(216, 255, 255)); //sf::Color::Transparent);// 
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(2);

	shape.setPosition(100, 100);
	//shape.setRotation(theta);

	int npoints = 5;
	makeRegularPolygon(npoints);

	shape.setPointCount(npoints);
	for (int i = 0; i < npoints; ++i)
	{
		shape.setPoint(i, points[i]);
	}
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

void ConvexPolygon::makeRegularPolygon(int nsides)
{
	using std::cos, std::sin;
		
	real r = 50;

	for (int i = 0; i < nsides; ++i)
	{
		real angle = (2 * i * pi) / nsides;
		points.push_back({ r * sin(angle), -r * cos(angle) });
	}
}
