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


	vec2 clipNormal = normalise(refEdge);
	real eps = 1e-5;
	
	bool OK1 = clip(-clipNormal, refPoint1, eps, refEdgeIndex, cp1, cp2);
	bool OK2 = clip(clipNormal, refPoint2, eps, ref->nextIndex(refEdgeIndex), cp1, cp2);

	// If clipping returns no valid points, then both contact points were outside the plane
	// This would suggest something went wrong with collision detection
	assert(OK1 && OK2); 


	n = ref->normal(refEdgeIndex);
	real p1 = dot(cp1.point - refPoint1, n);
	real p2 = dot(cp2.point - refPoint1, n);

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
		cp.point -= cp.penetration * n;
	}

	inCrossFactors.resize(ncp);
	itCrossFactors.resize(ncp);
	rnCrossFactors.resize(ncp);
	rtCrossFactors.resize(ncp);

	// Sort by index off incident point to ensure a consistent ordering
	// Note - clipping process above should give consistent order anyway?
	//std::sort(contactPoints.begin(), contactPoints.end(),
	//	[](const ContactPoint& cp1, const ContactPoint& cp2) 
	//	{ return cp1.incPointIndex < cp2.incPointIndex; }); 
}

PolyPolyContact::~PolyPolyContact()
{
	//std::cout << "~PolyPolyContact()\n";
}

void PolyPolyContact::warmStart()
{
	for (int i = 0; i < ncp; ++i)
	{
		const ContactPoint& cp = contactPoints[i];

		inc->applyDeltaVel(n * inc->mInv * cp.lambda + t * inc->mInv * cp.fLambda,
			inCrossFactors[i] * inc->IInv * cp.lambda + itCrossFactors[i] * inc->IInv * cp.fLambda);

		ref->applyDeltaVel(-n * ref->mInv * cp.lambda - t * ref->mInv * cp.fLambda,
			-rnCrossFactors[i] * ref->IInv * cp.lambda - rtCrossFactors[i] * ref->IInv * cp.fLambda);
	}
}

void PolyPolyContact::correctVel()
{
	for (int i = 0; i < ncp; ++i)
	{
		ContactPoint& cp = contactPoints[i];

		// Friction
		real massFactor = inc->mInv + ref->mInv + inc->IInv * std::pow(itCrossFactors[i], 2) + ref->IInv * std::pow(rtCrossFactors[i], 2);
		real vDotGradC = dot(inc->velocity() - ref->velocity(), t) + itCrossFactors[i] * inc->angVel() - rtCrossFactors[i] * ref->angVel();

		real dfLambda = 0;
		if (massFactor != 0)
		{
			dfLambda = -(vDotGradC) / massFactor;
			dfLambda = std::clamp(cp.fLambda + dfLambda, -mu * cp.lambda, mu * cp.lambda) - cp.fLambda;
		}

		cp.fLambda += dfLambda;

		inc->applyDeltaVel(t * inc->mInv * dfLambda, itCrossFactors[i] * inc->IInv * dfLambda);
		ref->applyDeltaVel(-t * ref->mInv * dfLambda, -rtCrossFactors[i] * ref->IInv * dfLambda);
	}

	for (int i = 0; i < ncp; ++i)
	{
		// TODO: add restitution

		ContactPoint& cp = contactPoints[i];

		real massFactor = inc->mInv + ref->mInv + inc->IInv * std::pow(inCrossFactors[i], 2) + ref->IInv * std::pow(rnCrossFactors[i], 2);
		real vDotGradC = dot(inc->velocity() - ref->velocity(), n) + inCrossFactors[i] * inc->angVel() - rnCrossFactors[i] * ref->angVel();
		
		real dLambda = 0;
		if (massFactor != 0)
		{
			dLambda = -(vDotGradC) / massFactor;
			dLambda = std::max(cp.lambda + dLambda, static_cast<real>(0)) - cp.lambda;
		}

		cp.lambda += dLambda;

		inc->applyDeltaVel(n * inc->mInv * dLambda, inCrossFactors[i] * inc->IInv * dLambda);
		ref->applyDeltaVel(-n * ref->mInv * dLambda, -rnCrossFactors[i] * ref->IInv * dLambda);
	}
}

void PolyPolyContact::correctPos()
{
	rebuild();

	bool blockSolve = false;

	if (blockSolve && ncp == 2)
	{
		const vec2 c1 = contactPoints[0].point;
		const vec2 c2 = contactPoints[1].point;

		const vec2 ri = inc->position();
		const vec2 rr = ref->position();

		const vec2 n = ref->normal(refEdgeIndex);

		real iCross1 = zcross(c1 - ri, n);
		real iCross2 = zcross(c2 - ri, n);
		real rCross1 = zcross(c1 - rr, n);
		real rCross2 = zcross(c2 - rr, n);

		real slop = 0.005;
		real C1 = std::min(contactPoints[0].penetration + slop, static_cast<real>(0));
		real C2 = std::min(contactPoints[1].penetration + slop, static_cast<real>(0));

		real A11 = inc->mInv + ref->mInv + inc->IInv * iCross1 * iCross1 + ref->IInv * rCross1 * rCross1;

		real A12 = inc->mInv + ref->mInv + inc->IInv * iCross1 * iCross2 + ref->IInv * rCross1 * rCross2;

		real A22 = inc->mInv + ref->mInv + inc->IInv * iCross2 * iCross2 + ref->IInv * rCross2 * rCross2;

		real A21 = A12;

		real det = A11 * A22 - A21 * A12;



		real beta = 0.3;

		real alpha1 = -beta * C1;
		real alpha2 = -beta * C2;

		

		real lam1 = (A22 * alpha1 - A12 * alpha2) / det;
		real lam2 = (A11 * alpha2 - A21 * alpha1) / det;

		inc->applyDeltaPos(n * inc->mInv * lam1, iCross1 * inc->IInv * lam1);
		ref->applyDeltaPos(-n * ref->mInv * lam1, -rCross1 * ref->IInv * lam1);

		inc->applyDeltaPos(n * inc->mInv * lam2, iCross2 * inc->IInv * lam2);
		ref->applyDeltaPos(-n * ref->mInv * lam2, -rCross1 * ref->IInv * lam2);

		rebuild();
		return;
	}

	for (auto& cp : contactPoints)
	{
		// TODO: precompute these before this function is called?

		const vec2 c = cp.point;
		const vec2 ri = inc->position();
		const vec2 rr = ref->position();
		const vec2 n = ref->normal(refEdgeIndex);

		real iCross = zcross(c - ri, n);
		real rCross = zcross(c - rr, n);

		real massFactor = inc->mInv + ref->mInv + inc->IInv * iCross * iCross + ref->IInv * rCross * rCross;
		
		real C = std::min(cp.penetration + slop, static_cast<real>(0));


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
	

	// Assumes that the contact points are ordered consistently in both constraints,
	// i.e. cannot have 0 matching with 1 and 1 matching with 0
	if (ncp == 1)
	{
		return cp[0].matches(cpOther[0]);
	}
	else
	{
		return cp[0].matches(cpOther[0]) && cp[1].matches(cpOther[1]);
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

void PolyPolyContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other will be left in an invalid state

	PolyPolyContact* ppOther = static_cast<PolyPolyContact*>(other);

	for (int i = 0; i < ncp; ++i)
	{
		ppOther->contactPoints[i].lambda = contactPoints[i].lambda;
		ppOther->contactPoints[i].fLambda = contactPoints[i].fLambda;
	}

	contactPoints = std::move(ppOther->contactPoints);
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

void PolyPolyContact::updateCache()
{
	n = ref->normal(refEdgeIndex);
	t = perp(n);

	vec2 iRelPos, rRelPos;
	for (int i = 0; i < ncp; ++i)
	{
		iRelPos = contactPoints[i].point - inc->position();
		rRelPos = contactPoints[i].point - ref->position();

		inCrossFactors[i] = zcross(iRelPos, n);
		itCrossFactors[i] = zcross(iRelPos, t);
		rnCrossFactors[i] = zcross(rRelPos, n);
		rtCrossFactors[i] = zcross(rRelPos, t);
	}
}
