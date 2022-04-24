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

	//TODO: Make the clipping code more concise?

	ContactPoint cp1, cp2;
	cp1.incPointIndex = incEdgeIndex;
	cp1.refEdgeIndex = refEdgeIndex;
	cp1.point = incPoint1;

	cp2.incPointIndex = inc->nextIndex(incEdgeIndex);
	cp2.refEdgeIndex = refEdgeIndex;
	cp2.point = incPoint2;

	vec2 n = normalise(refEdge);
	
	// TODO: handle cases where one of the clips returns 0 or 1 points, though in practice
	// this situation shouldn't arise
	// (0 points -> 0 contact points)
	// (1 point -> only this point is a contact point)

	real eps = 1e-3;
	auto [valid11, valid12] = clip(-n, refPoint1, eps, refEdgeIndex, cp1, cp2);
	auto [valid21, valid22] = clip(n, refPoint2, eps, ref->nextIndex(refEdgeIndex), cp1, cp2);

	assert(valid11 && valid12 && valid21 && valid22);

	vec2 normal = ref->normal(refEdgeIndex);

	if (dot(cp1.point - refPoint1, normal) <= 0)
	{
		cp1.penetration = dot(cp1.point - refPoint1, normal);
		contactPoints.push_back(cp1);
	}

	if (dot(cp2.point - refPoint1, normal) <= 0)
	{
		cp2.penetration = dot(cp2.point - refPoint1, normal);
		contactPoints.push_back(cp2);
	}


	ncp = contactPoints.size();

	assert(ncp == 1 || ncp == 2);

	// Project contact points onto the reference edge
	for (auto& cp : contactPoints)
	{
		cp.point -= cp.penetration * normal;
	}

	//for (int i = 0; i < ncp; ++i)
	//{
	//	rebuildPoint(i);
	//}
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

bool PolyPolyContact::matches(const PolyPolyContact* other) const
{
	if (ref != other->ref || inc != other->inc || ncp != other->ncp)
	{
		return false;
	}

	const auto& cp = contactPoints;
	const auto& cpOther = other->contactPoints;

	assert(ncp == 1 || ncp == 2);
	
	if (ncp == 1)
	{
		return cp[0].matches(cpOther[0]);
	}
	else // if (ncp == 2)
	{
		// TODO: Is it actually possible to have 0<->1 and 1<->0?
		return ((cp[0].matches(cpOther[0]) && cp[1].matches(cpOther[1]))
				|| (cp[0].matches(cpOther[1]) && cp[1].matches(cpOther[0])));
	}
}

void PolyPolyContact::rebuild()
{
	// TODO: rebuild from another PolyPolyContact
	for (int i = 0; i < ncp; ++i)
	{
		rebuildPoint(i);
	}
}

void PolyPolyContact::rebuildPoint(int i)
{
	ContactPoint& cp = contactPoints[i];

	vec2 normal = ref->normal(refEdgeIndex);
	vec2 refPoint = ref->vertex(refEdgeIndex);

	if (cp.clippedAgainstPoint == -1)
	{
		// Wasn't clipped
		cp.point = inc->vertex(cp.incPointIndex);
	}
	else
	{
		// Find the point where the incident edge crosses the clip plane
		vec2 p = inc->edge(incEdgeIndex);
		vec2 q = normal;

		vec2 a = inc->vertex(cp.incPointIndex);
		vec2 b = ref->vertex(cp.clippedAgainstPoint);

		// TODO: Can we do this with dot products?
		vec2 intersect = a + p * zcross(q, b - a) / zcross(q, p);
		cp.point = intersect;
	}

	cp.penetration = dot(cp.point - refPoint, normal);

	// Project onto reference edge
	cp.point -= cp.penetration * normal;
}
