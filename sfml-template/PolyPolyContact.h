#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"
#include "Edge.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps);

	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override;
	bool matches(const CircleCircleContact* other) const override { return false; }
	bool matches(const PolyCircleContact* other) const override { return false; }

private:
	void initPoints() override;
	void rebuildPoint(ContactPoint& cp) override;
	void updateNormal() override;

	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	// TODO: probably don't need to store these
	const Edge* refEdge = nullptr;
	const Edge* incEdge = nullptr;

	// TODO: these could be in the base class?
	vec2 localNormal;
	vec2 localRefPoint;

	void checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps);
};

