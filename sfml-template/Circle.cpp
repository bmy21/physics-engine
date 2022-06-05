#include "Circle.h"

Circle::Circle(const PhysicsSettings& ps, real rad, real mInv):
	rad(rad),
	RigidBody(ps, mInv)
{
	IInv = 2 * mInv / (rad * rad);

	initShape();
}

void Circle::update(real dt)
{

}

void Circle::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateAngle(fraction);

	shape.setPosition(ipos * pixPerUnit);
	shape.setRotation(itheta);

	// TODO: Ideally, don't call these every frame 
	shape.setOrigin(rad * pixPerUnit, rad * pixPerUnit);
	shape.setRadius(rad * pixPerUnit);

	vec2 radiusVector = { rad * std::cos(itheta), rad * std::sin(itheta) };

	drawThickLine(window, 
		ipos * pixPerUnit, 
		(ipos + radiusVector) * pixPerUnit, 
		1, 
		sf::Color::Black);

	if (debug && text)
	{
		
	}

	window.draw(shape);
}

std::unique_ptr<ContactConstraint> Circle::checkCollision(Circle* other)
{
	if (magnitude(other->position() - position()) < rad + other->rad)
	{
		return std::make_unique<CircleCircleContact>(this, other, ps);
	}
	else
	{
		return nullptr;
	}
}

bool Circle::pointInside(const vec2& p) const
{
	return magnitude(p - position()) < rad;
}

vec2 Circle::furthestPoint(const vec2& d)
{
	return position() + d * rad;
}

void Circle::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);
	shape.setPointCount(60);
}
