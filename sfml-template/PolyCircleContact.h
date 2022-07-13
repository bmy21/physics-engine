#pragma once
#include "ContactConstraint.h"
#include "Circle.h"
#include "ConvexPolygon.h"

class PolyCircleContact : public ContactConstraint
{
public:
	PolyCircleContact(ConvexPolygon* p, Circle* c, const vec2& localNormal, const vec2& localRefPoint, 
		Voronoi region, const PhysicsSettings& ps);

private:
	void initPoints() override;
	void rebuildPoint(ContactPoint& cp) override;
	void updateNormal() override;

	void setRollingFriction();

	const ConvexPolygon* const p;
	const Circle* const c;

	vec2 localNormal;
	vec2 localRefPoint;
	Voronoi region;
};