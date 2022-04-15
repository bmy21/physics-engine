#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "ConvexPolygon.h"

class PolyPolyContact : public ContactConstraint
{
public:
	PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex, int incPointIndex);
	~PolyPolyContact();

	void correctVel() override;
	void correctPos() override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction) override;

private:
	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	int ncp = -1;
	int refEdgeIndex = -1;
	int incEdgeIndex = -1;
	int incPointIndex = -1;

	std::vector<ContactPoint> contactPoints;
};

