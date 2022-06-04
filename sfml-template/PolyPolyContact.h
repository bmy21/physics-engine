#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"
#include "Edge.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps);

	void warmStart() override;
	void correctVel() override;
	void correctPos() override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override;
	bool matches(const CircleCircleContact* other) const override { return false; }

	void rebuild() override;
	void rebuildFrom(ContactConstraint* other) override;

	void updateCache() override;

private:
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;

	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	const Edge* refEdge = nullptr;
	const Edge* incEdge = nullptr;

	std::vector<ContactPoint> contactPoints;
	int ncp = -1;
	
	// Cached data for simultaneous solution
	real A12 = 0, det = 0, norm = 0;

	void checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps);
};

