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
	cp1.incPointIndex = incEdgeIndex;
	cp1.refEdgeIndex = refEdgeIndex;
	cp1.point = incPoint1;

	cp2.incPointIndex = inc->nextIndex(incEdgeIndex);
	cp2.refEdgeIndex = refEdgeIndex;
	cp2.point = incPoint2;


	vec2 n = normalise(refEdge);
	real eps = 1e-5;
	
	bool OK1 = clip(-n, refPoint1, eps, refEdgeIndex, cp1, cp2);
	bool OK2 = clip(n, refPoint2, eps, ref->nextIndex(refEdgeIndex), cp1, cp2);

	// If clipping returns no valid points, then both contact points were outside the plane
	// This would suggest something went wrong with collision detection
	assert(OK1 && OK2); 


	vec2 normal = ref->normal(refEdgeIndex);


	real p1 = dot(cp1.point - refPoint1, normal);
	real p2 = dot(cp2.point - refPoint1, normal);

	if (p1 <= 0)
	{
		cp1.penetration = p1;
		contactPoints.push_back(cp1);
	}

	if (p2 <= 0)
	{
		cp2.penetration = p2;
		contactPoints.push_back(cp2);
	}


	ncp = contactPoints.size();

	assert(ncp == 1 || ncp == 2);

	// Project contact points onto the reference edge
	for (auto& cp : contactPoints)
	{
		cp.point -= cp.penetration * normal;
	}
}

PolyPolyContact::~PolyPolyContact()
{
	//std::cout << "~PolyPolyContact()\n";
}

void PolyPolyContact::warmStart()
{
	for (auto& cp : contactPoints)
	{
		const vec2& c = cp.point;
		const vec2 ri = inc->position();
		const vec2 rr = ref->position();
		const vec2 n = ref->normal(refEdgeIndex);

		real iCross = zcross(c - ri, n);
		real rCross = zcross(c - rr, n);

		inc->applyDeltaVel(n * inc->mInv * cp.lambda, iCross * inc->IInv * cp.lambda);
		ref->applyDeltaVel(-n * ref->mInv * cp.lambda, -rCross * ref->IInv * cp.lambda);
	}
}

void PolyPolyContact::correctVel()
{
	for (auto& cp : contactPoints)
	{
		// TODO: precompute these before this function is called?

		// TODO: add restitution

		const vec2& c = cp.point;
		const vec2 ri = inc->position();
		const vec2 rr = ref->position();
		const vec2 n = ref->normal(refEdgeIndex);

		real iCross = zcross(c - ri, n);
		real rCross = zcross(c - rr, n);

		real massFactor = inc->mInv + ref->mInv + inc->IInv * iCross * iCross + ref->IInv * rCross * rCross;
		real vDotGradC = dot(inc->velocity() - ref->velocity(), n) + iCross * inc->angVel() - rCross * ref->angVel();
		
		//real C = dot(c - ref->vertex(refEdgeIndex), n);

		real dLambda = 0;
		if (massFactor != 0)
		{
			dLambda = -(vDotGradC) / massFactor;
			dLambda = std::max(cp.lambda + dLambda, static_cast<real>(0)) - cp.lambda;
		}

		inc->applyDeltaVel(n * inc->mInv * dLambda, iCross * inc->IInv * dLambda);
		ref->applyDeltaVel(-n * ref->mInv * dLambda, -rCross * ref->IInv * dLambda);

		cp.lambda += dLambda;
	}
}

void PolyPolyContact::correctPos()
{
	for (auto& cp : contactPoints)
	{
		// TODO: precompute these before this function is called?

		// TODO: add slop

		const vec2 c = cp.point;
		const vec2 ri = inc->position();
		const vec2 rr = ref->position();
		const vec2 n = ref->normal(refEdgeIndex);

		real iCross = zcross(c - ri, n);
		real rCross = zcross(c - rr, n);

		real massFactor = inc->mInv + ref->mInv + inc->IInv * iCross * iCross + ref->IInv * rCross * rCross;
		
		real slop = 0.001;
		real C = std::min(cp.penetration + slop, static_cast<real>(0));

		real beta = 0.5;

		real dLambda = 0;
		if (massFactor != 0)
		{
			dLambda = -beta * C / massFactor;
		}
		
		inc->applyDeltaPos(n * inc->mInv * dLambda, iCross * inc->IInv * dLambda);
		ref->applyDeltaPos(-n * ref->mInv * dLambda, -rCross * ref->IInv * dLambda);

		rebuild();
	}
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	// std::cout << numPersist << '\n';

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

	auto& cp = contactPoints;
	auto& cpOther = other->contactPoints;

	assert(ncp == 1 || ncp == 2);
	
	// TODO: simplify with a findMatchingIndex function?
	// TODO: order by depth so that we don't need matchingIndex?
	//		 or somehow order according to id, to remove any ambiguity
	if (ncp == 1 && cp[0].matches(cpOther[0]))
	{
		cp[0].matchingIndex = cpOther[0].matchingIndex = 0;
		return true;
	}
	else if (cp[0].matches(cpOther[0]) && cp[1].matches(cpOther[1]))
	{
		cp[0].matchingIndex = cpOther[0].matchingIndex = 0;
		cp[1].matchingIndex = cpOther[1].matchingIndex = 1;
		return true;
	}
	else if (cp[0].matches(cpOther[1]) && cp[1].matches(cpOther[0]))
	{
		cp[0].matchingIndex = cpOther[0].matchingIndex = 1;
		cp[1].matchingIndex = cpOther[1].matchingIndex = 0;
		return true;
	}

	return false;
}

void PolyPolyContact::rebuild()
{
	// TODO: rebuild from another PolyPolyContact
	for (int i = 0; i < ncp; ++i)
	{
		rebuildPoint(i);
	}
}

void PolyPolyContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other will be left in an invalid state


	// TODO: order of points matters for warm starting!
	// Which point gets which accumulated lambda?
	PolyPolyContact* ppOther = static_cast<PolyPolyContact*>(other);

	for (auto& cpOther : ppOther->contactPoints)
	{
		cpOther.lambda = contactPoints[cpOther.matchingIndex].lambda;
	}

	contactPoints = std::move(ppOther->contactPoints);


	//std::cout << contactPoints[0].lambda << '\n';
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

		// TODO: Quicker to do this with dot products?
		vec2 intersect = a + p * zcross(q, b - a) / zcross(q, p);
		cp.point = intersect;
	}

	cp.penetration = dot(cp.point - refPoint, normal);

	// Project onto reference edge
	cp.point -= cp.penetration * normal;
}
