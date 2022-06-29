#include "Circle.h"
#include "ConvexPolygon.h"

Circle::Circle(const PhysicsSettings& ps, real rad, real mInv):
	rad(rad),
	RigidBody(ps, mInv)
{
	IInv = 2 * mInv / (rad * rad);

	initShape();
}

void Circle::draw(sf::RenderWindow& window, real fraction, bool debug, sf::Text* text)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateAngle(fraction);

	shape.setPosition(ipos * ps.pixPerUnit);
	shape.setRotation(itheta);

	//vec2 radiusVector = { rad * std::cos(itheta), rad * std::sin(itheta) };

	

	if (debug && text)
	{
		
	}
	 
	window.draw(shape);

	line.setPosition(ipos * ps.pixPerUnit);
	line.setRotation(itheta);

	window.draw(line);

	//drawThickLine(window,
	//	ipos * pixPerUnit,
	//	(ipos + radiusVector) * pixPerUnit,
	//	1,
	//	sf::Color::Black);
}

std::unique_ptr<ContactConstraint> Circle::checkCollision(ConvexPolygon* other)
{
	return other->checkCollision(this);
}

std::unique_ptr<ContactConstraint> Circle::checkCollision(Circle* other)
{
	real radiusSum = rad + other->rad;
	if (magSquared(other->position() - position()) < radiusSum * radiusSum)
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
	return magSquared(p - position()) < rad * rad;
}

void Circle::updateAABB()
{
	aabb.lower = { position().x - rad, position().y - rad };
	aabb.upper = { position().x + rad, position().y + rad };
}

void Circle::updateFatAABB(real w)
{
	aabbFat.lower = { position().x - rad - w, position().y - rad - w };
	aabbFat.upper = { position().x + rad + w, position().y + rad + w };
}

vec2 Circle::furthestPoint(const vec2& d)
{
	return position() + d * rad;
}

void Circle::initShape()
{
	sf::Color col(196, 250, 248);
	shape.setFillColor(col);
	//shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);
	shape.setPointCount(60);

	// TODO: May need to update these later if pixPerUnit is changed
	shape.setOrigin(rad * ps.pixPerUnit, rad * ps.pixPerUnit);
	shape.setRadius(rad * ps.pixPerUnit);

	line.setFillColor(sf::Color::Black);
	line.setOrigin(0, lineThicknessPix / 2);
	line.setSize({ rad * ps.pixPerUnit, lineThicknessPix });
}
