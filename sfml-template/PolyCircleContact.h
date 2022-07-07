#pragma once
#include "ContactConstraint.h"
#include "Circle.h"
#include "ConvexPolygon.h"

class PolyCircleContact : public ContactConstraint
{
public:
	PolyCircleContact(ConvexPolygon* p, Circle* c, const vec2& localNormal, const vec2& localRefPoint, 
		Voronoi region, const PhysicsSettings& ps);

	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override { return false;  }
	bool matches(const CircleCircleContact* other) const override { return false; }
	bool matches(const PolyCircleContact* other) const override;

private:
	void initPoints() override;
	void rebuildPoint(ContactPoint& cp) override;
	void updateNormal() override;

	void setRollingFriction();

	// TODO: make these const pointers (in other CCs too)
	const ConvexPolygon* const p;
	const Circle* const c;

	vec2 localNormal;
	vec2 localRefPoint;
	Voronoi region;
};