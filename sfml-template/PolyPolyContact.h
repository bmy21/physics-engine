#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex);
	~PolyPolyContact();

	void warmStart() override;
	void correctVel() override;
	void correctPos() override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override;

	void rebuild() override;
	void rebuildFrom(ContactConstraint* other) override;

private:
	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	void rebuildPoint(int i);

	int ncp = -1;
	int refEdgeIndex = -1;
	int incEdgeIndex = -1;

	std::vector<ContactPoint> contactPoints;
};

