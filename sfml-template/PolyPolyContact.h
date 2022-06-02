#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex);

	void onInit() override;

	void warmStart() override;
	void correctVel() override;
	void correctPos() override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override;

	void rebuild() override;
	void rebuildFrom(ContactConstraint* other) override;

	void updateCache() override;

private:
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;

	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	int refEdgeIndex = -1;
	int incEdgeIndex = -1;

	std::vector<real> vRelTarget;

	std::vector<ContactPoint> contactPoints;
	int ncp = -1;
	
	// Geometric data - cached to avoid recomputation
	vec2 n, t;
	std::vector<real> inCrossFactors, rnCrossFactors;
	std::vector<real> itCrossFactors, rtCrossFactors;
	std::vector<real> nMassFactors, tMassFactors;
	real A12 = 0, det = 0, norm = 0;

	void rebuildPoint(int i);
	void checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps);
};

