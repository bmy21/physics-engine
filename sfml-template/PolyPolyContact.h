#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"
#include "Edge.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps);

private:
	void initPoints() override;
	void rebuildPoint(ContactPoint& cp) override;
	void updateNormal() override;

	const ConvexPolygon* const ref;
	const ConvexPolygon* const inc;

	// Store incident and reference edges for use in initPoints
	const Edge* refEdge = nullptr;
	const Edge* incEdge = nullptr;

	vec2 localNormal;
	vec2 localRefPoint;

	void checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps);
};

