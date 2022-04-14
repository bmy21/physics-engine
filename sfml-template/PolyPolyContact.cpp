#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex):
	ref(ref),
	inc(inc),
	refEdgeIndex(refEdgeIndex),
	incEdgeIndex(incEdgeIndex)
{

}

PolyPolyContact::~PolyPolyContact()
{
	//std::cout << "~PolyPolyContact()\n";
}

void PolyPolyContact::correctVel()
{
}

void PolyPolyContact::correctPos()
{
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction)
{
	vec2 refPoint1 = ref->transformedPoint(refEdgeIndex) * pixPerUnit;
	vec2 refPoint2 = refPoint1 + ref->transformedEdge(refEdgeIndex) * pixPerUnit;
	drawLine(window, refPoint1, refPoint2, sf::Color::Red);
	
	vec2 incPoint1 = inc->transformedPoint(incEdgeIndex) * pixPerUnit;
	vec2 incPoint2 = incPoint1 + inc->transformedEdge(incEdgeIndex) * pixPerUnit;
	drawLine(window, incPoint1, incPoint2, sf::Color::Green);
}
