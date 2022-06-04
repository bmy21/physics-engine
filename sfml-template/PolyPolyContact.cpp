#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps):
	ref(ref),
	inc(inc),
	refEdge(refEdge),
	incEdge(incEdge),
	ContactConstraint(ps, ref, inc)
{
	vec2 refPoint1 = refEdge->point1();
	vec2 refPoint2 = refEdge->point2();

	vec2 incPoint1 = incEdge->point1();
	vec2 incPoint2 = incEdge->point2();

	ContactPoint cp1, cp2;
	cp1.incPointIndex = incEdge->v1index();
	cp1.refEdgeIndex = refEdge->index();
	cp1.point = incPoint1;

	cp2.incPointIndex = incEdge->v2index();
	cp2.refEdgeIndex = refEdge->index();
	cp2.point = incPoint2;

	vec2 clipNormal = normalise(refEdge->global());
	bool OK1 = clip(-clipNormal, refPoint1, ps.clipPlaneEpsilon, refEdge->v1index(), cp1, cp2);
	bool OK2 = clip(clipNormal, refPoint2, ps.clipPlaneEpsilon, refEdge->v2index(), cp1, cp2);

	// If clipping returns no valid points, then both contact points were outside the plane
	// This would suggest something went wrong with collision detection
	// assert(OK1 && OK2); 

	n = refEdge->normal();

	checkAndAddPoint(cp1, refPoint1, ps.clipPlaneEpsilon);
	checkAndAddPoint(cp2, refPoint1, ps.clipPlaneEpsilon);

	ncp = contactPoints.size();

	// Sort by index off incident point to ensure a consistent ordering
	// Note - clipping process above should give consistent order anyway?
	// std::sort(contactPoints.begin(), contactPoints.end(),
	//	[](const ContactPoint& cp1, const ContactPoint& cp2) 
	//	{ return cp1.incPointIndex < cp2.incPointIndex; }); 


	// Store target relative velocities
	for (auto& cp : contactPoints)
	{
		real vRel = dot(inc->pointVel(cp.point) - ref->pointVel(cp.point), n);
		cp.vRelTarget = vRel < -ps.vRelThreshold ? -e * vRel : 0;
	}
}

void PolyPolyContact::warmStart()
{
	for (auto& cp : contactPoints)
	{
		warmStartPoint(cp);
	}
}

void PolyPolyContact::correctVel()
{
	for (auto& cp : contactPoints)
	{
		solvePointFriction(cp);
	}

	// Try simultaneous solution of normal velocities first
	if (ncp == 2 && ps.simulSolveVel && wellConditionedVel)
	{
		ContactPoint& cp1 = contactPoints[0];
		ContactPoint& cp2 = contactPoints[1];

		real alpha1 = cp1.vRelTarget - (dot(inc->velocity() - ref->velocity(), n) + cp1.nCrossFactor2 * inc->angVel() - cp1.nCrossFactor1 * ref->angVel());
		real alpha2 = cp2.vRelTarget - (dot(inc->velocity() - ref->velocity(), n) + cp2.nCrossFactor2 * inc->angVel() - cp2.nCrossFactor1 * ref->angVel());

		real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
		real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;


		if (cp1.lambda + lam1 >= 0 && cp2.lambda + lam2 >= 0)
		{
			inc->applyDeltaVel(n * inc->mInv * (lam1 + lam2), inc->IInv * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));
			ref->applyDeltaVel(-n * ref->mInv * (lam1 + lam2), ref->IInv * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));

			cp1.lambda += lam1;
			cp2.lambda += lam2;

			return;
		}
	}
		
	// At this point, simultaneous solution failed, either because the condition number was too high or 
	// because one of the accumulated impulses would become negative. So, resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		solvePointVel(cp);
	}
}

void PolyPolyContact::correctPos()
{
	// Try simultaneous solution first
	if (ps.simulSolvePos && ncp == 2)
	{
		rebuild();
		updateCache();

		if (wellConditionedPos)
		{
			ContactPoint& cp1 = contactPoints[0];
			ContactPoint& cp2 = contactPoints[1];

			real C1 = std::min(cp1.penetration + ps.slop, static_cast<real>(0));
			real C2 = std::min(cp2.penetration + ps.slop, static_cast<real>(0));

			real alpha1 = -ps.beta * C1;
			real alpha2 = -ps.beta * C2;

			real lam1 = (cp2.nMassFactor * alpha1 - A12 * alpha2) / det;
			real lam2 = (cp1.nMassFactor * alpha2 - A12 * alpha1) / det;

			inc->applyDeltaPos(n * inc->mInv * (lam1 + lam2), inc->IInv * (cp1.nCrossFactor2 * lam1 + cp2.nCrossFactor2 * lam2));
			ref->applyDeltaPos(-n * ref->mInv * (lam1 + lam2), ref->IInv * (-cp1.nCrossFactor1 * lam1 - cp2.nCrossFactor1 * lam2));

			return;
		}
	}

	// At this point, the condition number was too high, so resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		solvePointPos(cp);
	}
}

void PolyPolyContact::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	// std::cout << numPersist << '\n';

	vec2 refPoint1 = refEdge->point1() * pixPerUnit;
	vec2 refPoint2 = refEdge->point2() * pixPerUnit;

	vec2 incPoint1 = incEdge->point1() * pixPerUnit;
	vec2 incPoint2 = incEdge->point2() * pixPerUnit;

	drawThickLine(window, refPoint1, refPoint2, 3, sf::Color::Red);
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
	if (ref != other->ref || inc != other->inc || ncp != other->ncp || ncp == 0)
	{
		return false;
	}

	auto& cp = contactPoints;
	auto& cpOther = other->contactPoints;
	
	// Assumes that the contact points are ordered consistently in both constraints,
	// i.e. cannot have 0 matching with 1 and 1 matching with 0
	if (ncp == 1)
	{
		return cp[0].matches(cpOther[0]);
	}
	else if (ncp == 2)
	{
		return cp[0].matches(cpOther[0]) && cp[1].matches(cpOther[1]);
	}
}

void PolyPolyContact::rebuild()
{
	vec2 normal = refEdge->normal();
	vec2 refPoint = refEdge->point1();

	for (auto& cp : contactPoints)
	{	
		if (cp.clippedAgainstPoint == -1)
		{
			// Wasn't clipped
			cp.point = inc->vertex(cp.incPointIndex);
		}
		else
		{
			// Find the point where the incident edge crosses the clip plane
			vec2 p = incEdge->global();
			vec2 q = normal;

			vec2 a = inc->vertex(cp.incPointIndex);
			vec2 b = ref->vertex(cp.clippedAgainstPoint);

			// TODO: Quicker to do this with dot products?
			assert(zcross(q, p) != 0);
			vec2 intersect = a + p * zcross(q, b - a) / zcross(q, p);
			cp.point = intersect;
		}

		cp.penetration = dot(cp.point - refPoint, normal);

		// Project onto reference edge
		cp.point -= cp.penetration * normal;
	}
}

void PolyPolyContact::rebuildFrom(ContactConstraint* other)
{
	// This function should only be called if *other is known to match *this
	// *other may be left in an invalid state

	PolyPolyContact* ppOther = static_cast<PolyPolyContact*>(other);

	for (int i = 0; i < ncp; ++i)
	{
		// Updated vRelTarget is already stored in ppOther->contactPoints
		ppOther->contactPoints[i].lambda = contactPoints[i].lambda;
		ppOther->contactPoints[i].fLambda = contactPoints[i].fLambda;
	}

	contactPoints = std::move(ppOther->contactPoints);

	refEdge = ppOther->refEdge;
	incEdge = ppOther->incEdge;
}

void PolyPolyContact::checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps)
{
	// If cp lies inside the reference edge, store its penetration, project it into the edge, 
	// and add it to the contactPoints vector.

	real p = dot(cp.point - ref, n);

	if (p <= eps)
	{
		cp.penetration = p;
		cp.point -= cp.penetration * n;
		contactPoints.push_back(cp);
	}
}

void PolyPolyContact::updateCache()
{
	n = refEdge->normal();
	t = perp(n);

	for (auto& cp : contactPoints)
	{
		updatePointCache(cp);
	}

	if ((ps.simulSolveVel || ps.simulSolvePos) && ncp == 2)
	{
		ContactPoint& cp1 = contactPoints[0];
		ContactPoint& cp2 = contactPoints[1];

		A12 = inc->mInv + ref->mInv + inc->IInv * cp1.nCrossFactor2 * cp2.nCrossFactor2 + ref->IInv * cp1.nCrossFactor1 * cp2.nCrossFactor1;
		det = cp1.nMassFactor * cp2.nMassFactor - A12 * A12;
		norm = std::max(cp1.nMassFactor, cp2.nMassFactor) + std::abs(A12);
		real normSquared = norm * norm;

		// Is the condition number less than the threshold?
		wellConditionedVel = normSquared < ps.maxCondVel * det;
		wellConditionedPos = normSquared < ps.maxCondPos * det;
	}
}
