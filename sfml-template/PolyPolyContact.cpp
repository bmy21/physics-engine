#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, int refEdgeIndex, int incEdgeIndex):
	ref(ref),
	inc(inc),
	refEdgeIndex(refEdgeIndex),
	incEdgeIndex(incEdgeIndex)
{
	vec2 refEdge = ref->edge(refEdgeIndex);
	vec2 refPoint1 = ref->vertex(refEdgeIndex);
	vec2 refPoint2 = refPoint1 + refEdge;

	vec2 incEdge = inc->edge(incEdgeIndex);
	vec2 incPoint1 = inc->vertex(incEdgeIndex);
	vec2 incPoint2 = incPoint1 + incEdge;

	ContactPoint cp1, cp2;
	cp1.pointIndex = incEdgeIndex;
	cp2.pointIndex = inc->nextIndex(incEdgeIndex);

	// Clip incident edge against first side of reference edge
	ClipType type1 = ClipType::Invalid;
	auto cpCoords = clip(-refEdge, refPoint1, incPoint1, incPoint2, type1);

	assert(type1 == ClipType::First
		|| type1 == ClipType::Second
		|| type1 == ClipType::None);

	if (type1 == ClipType::First)
	{
		cp1.clippedAgainstEdge = refEdgeIndex;
		cp1.clippedAgainstPoint = refEdgeIndex;
	}
	else if (type1 == ClipType::Second)
	{
		cp2.clippedAgainstEdge = refEdgeIndex;
		cp2.clippedAgainstPoint = refEdgeIndex;
	}
	

	// Clip again against other side edge
	ClipType type2 = ClipType::Invalid;
	cpCoords = clip(refEdge, refPoint2, cpCoords[0], cpCoords[1], type2);

	assert(type2 == ClipType::First
		|| type2 == ClipType::Second
		|| type2 == ClipType::None);

	if (type2 == ClipType::First)
	{
		cp1.clippedAgainstEdge = refEdgeIndex;
		cp1.clippedAgainstPoint = ref->nextIndex(refEdgeIndex);
	}
	else if (type2 == ClipType::Second)
	{
		cp2.clippedAgainstEdge = refEdgeIndex;
		cp2.clippedAgainstPoint = ref->nextIndex(refEdgeIndex);
	}
	
	
	vec2 normal = ref->normal(refEdgeIndex);

	if (dot(cpCoords[0] - refPoint1, normal) <= 0)
	{
		cp1.penetration = dot(cpCoords[0] - refPoint1, normal);
		cp1.point = cpCoords[0];
		contactPoints.push_back(cp1);
	}
	if (dot(cpCoords[1] - refPoint1, normal) <= 0)
	{
		cp2.penetration = dot(cpCoords[1] - refPoint1, normal);
		cp2.point = cpCoords[1];
		contactPoints.push_back(cp2);
	}

	ncp = contactPoints.size();
	assert(ncp == 1 || ncp == 2);

	// Project contact points onto the reference edge
	for (auto& cp : contactPoints)
	{
		cp.point -= cp.penetration * normal;
		
	}



	for (int i = 0; i < ncp; ++i)
	{
		rebuildPoint(i);
	}
}

PolyPolyContact::~PolyPolyContact()
{
	std::cout << "~PolyPolyContact()\n";
}

void PolyPolyContact::correctVel()
{
}

void PolyPolyContact::correctPos()
{
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	vec2 refPoint1 = ref->vertex(refEdgeIndex) * pixPerUnit;
	vec2 refPoint2 = refPoint1 + ref->edge(refEdgeIndex) * pixPerUnit;
	drawThickLine(window, refPoint1, refPoint2, 3, sf::Color::Red);
	
	vec2 incPoint1 = inc->vertex(incEdgeIndex) * pixPerUnit;
	vec2 incPoint2 = incPoint1 + inc->edge(incEdgeIndex) * pixPerUnit;
	drawThickLine(window, incPoint1, incPoint2, 3, sf::Color::Green);


	real rad = 5;
	sf::CircleShape circle(rad);
	circle.setOrigin(rad, rad);
	circle.setFillColor(sf::Color::Magenta);

	for (const auto& cp : contactPoints)
	{
		circle.setPosition(cp.point.x*pixPerUnit, cp.point.y*pixPerUnit);
		window.draw(circle);

		if (debug && text)
		{
			text->setCharacterSize(40);
			text->setFillColor(sf::Color::Magenta);

			text->setString("\n\n" + cp.idAsString());

			text->setPosition(cp.point*pixPerUnit);
			centre(*text);

			window.draw(*text);
		}
	}
}

void PolyPolyContact::rebuildPoint(int i)
{
	ContactPoint& cp = contactPoints[i];

	vec2 normal = ref->normal(refEdgeIndex);
	vec2 refPoint = ref->vertex(refEdgeIndex);

	if (cp.clippedAgainstEdge == -1)
	{
		// Wasn't clipped
		cp.point = inc->vertex(cp.pointIndex);
	}
	else
	{
		// Find the point where the incident edge crosses the clip plane
		vec2 p = inc->edge(incEdgeIndex);
		vec2 q = normal;

		vec2 a = inc->vertex(cp.pointIndex);
		vec2 b = ref->vertex(cp.clippedAgainstPoint);

		// TODO: Can we do this with dot products?
		vec2 intersect = a + p * zcross(q, b - a) / zcross(q, p);
		cp.point = intersect;
	}

	cp.penetration = dot(cp.point - refPoint, normal);

	// Project onto reference edge
	cp.point -= cp.penetration * normal;
}
