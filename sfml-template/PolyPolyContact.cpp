#include "PolyPolyContact.h"

PolyPolyContact::PolyPolyContact(ConvexPolygon* ref, ConvexPolygon* inc, const Edge* refEdge, const Edge* incEdge, const PhysicsSettings& ps):
	ref(ref),
	inc(inc),
	refEdge(refEdge),
	incEdge(incEdge),
	ContactConstraint(ps)
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

	inCrossFactors.resize(ncp);
	itCrossFactors.resize(ncp);
	rnCrossFactors.resize(ncp);
	rtCrossFactors.resize(ncp);

	nMassFactors.resize(ncp);
	tMassFactors.resize(ncp);


	// Sort by index off incident point to ensure a consistent ordering
	// Note - clipping process above should give consistent order anyway?
	// std::sort(contactPoints.begin(), contactPoints.end(),
	//	[](const ContactPoint& cp1, const ContactPoint& cp2) 
	//	{ return cp1.incPointIndex < cp2.incPointIndex; }); 


	// Store target relative velocities
	for (const auto& cp : contactPoints)
	{
		real vRel = dot(inc->pointVel(cp.point) - ref->pointVel(cp.point), n);

		//std::cout << vRel << "\n";

		//real rest = std::min(e * std::abs(vRel) / 50., 1.);

		vRelTarget.push_back(vRel < -ps.vRelThreshold ? -e * vRel : 0);
	}
}

void PolyPolyContact::warmStart()
{
	if (ps.warmStart)
	{
		for (int i = 0; i < ncp; ++i)
		{
			ContactPoint& cp = contactPoints[i];

			inc->applyDeltaVel(n * inc->mInv * cp.lambda + t * inc->mInv * cp.fLambda,
				inCrossFactors[i] * inc->IInv * cp.lambda + itCrossFactors[i] * inc->IInv * cp.fLambda);

			ref->applyDeltaVel(-n * ref->mInv * cp.lambda - t * ref->mInv * cp.fLambda,
				-rnCrossFactors[i] * ref->IInv * cp.lambda - rtCrossFactors[i] * ref->IInv * cp.fLambda);
		}
	}
	else
	{
		for (auto& cp : contactPoints)
		{
			cp.lambda = cp.fLambda = 0;
		}
	}
}

void PolyPolyContact::correctVel()
{
	for (int i = 0; i < ncp; ++i)
	{
		ContactPoint& cp = contactPoints[i];

		// Friction
		real vDotGradC = dot(inc->velocity() - ref->velocity(), t) + itCrossFactors[i] * inc->angVel() - rtCrossFactors[i] * ref->angVel();

		real dfLambda = 0;
		if (tMassFactors[i] != 0)
		{
			dfLambda = -(vDotGradC) / tMassFactors[i];
			dfLambda = std::clamp(cp.fLambda + dfLambda, -mu * cp.lambda, mu * cp.lambda) - cp.fLambda;
		}

		cp.fLambda += dfLambda;

		inc->applyDeltaVel(t * inc->mInv * dfLambda, itCrossFactors[i] * inc->IInv * dfLambda);
		ref->applyDeltaVel(-t * ref->mInv * dfLambda, -rtCrossFactors[i] * ref->IInv * dfLambda);
	}

	if (ncp == 2 && ps.simulSolveVel && wellConditionedVel)
	{
		real alpha1 = vRelTarget[0] - (dot(inc->velocity() - ref->velocity(), n) + inCrossFactors[0] * inc->angVel() - rnCrossFactors[0] * ref->angVel());
		real alpha2 = vRelTarget[1] - (dot(inc->velocity() - ref->velocity(), n) + inCrossFactors[1] * inc->angVel() - rnCrossFactors[1] * ref->angVel());

		real lam1 = (nMassFactors[1] * alpha1 - A12 * alpha2) / det;
		real lam2 = (nMassFactors[0] * alpha2 - A12 * alpha1) / det;


		if (contactPoints[0].lambda + lam1 >= 0 && contactPoints[1].lambda + lam2 >= 0)
		{
			inc->applyDeltaVel(n * inc->mInv * (lam1 + lam2), inc->IInv * (inCrossFactors[0] * lam1 + inCrossFactors[1] * lam2));
			ref->applyDeltaVel(-n * ref->mInv * (lam1 + lam2), ref->IInv * (-rnCrossFactors[0] * lam1 - rnCrossFactors[1] * lam2));

			contactPoints[0].lambda += lam1;
			contactPoints[1].lambda += lam2;

			return;
		}
	}
		
	// At this point, simultaneous solution failed, either because the condition number was too high or 
	// because one of the accumulated impulses would become negative. So, resort to the iterative solution.
	
	for (int i = 0; i < ncp; ++i)
	{
		ContactPoint& cp = contactPoints[i];
		
		real vDotGradC = dot(inc->velocity() - ref->velocity(), n) + inCrossFactors[i] * inc->angVel() - rnCrossFactors[i] * ref->angVel();

		real dLambda = 0;
		if (nMassFactors[i] != 0)
		{
			dLambda = (vRelTarget[i] - vDotGradC) / nMassFactors[i];
			dLambda = std::max(cp.lambda + dLambda, static_cast<real>(0)) - cp.lambda;
		}

		cp.lambda += dLambda;

		inc->applyDeltaVel(n * inc->mInv * dLambda, inCrossFactors[i] * inc->IInv * dLambda);
		ref->applyDeltaVel(-n * ref->mInv * dLambda, -rnCrossFactors[i] * ref->IInv * dLambda);
	}
}

void PolyPolyContact::correctPos()
{
	if (ps.simulSolvePos && ncp == 2)
	{
		rebuild();
		updateCache();

		if (wellConditionedPos)
		{
			real C1 = std::min(contactPoints[0].penetration + ps.slop, static_cast<real>(0));
			real C2 = std::min(contactPoints[1].penetration + ps.slop, static_cast<real>(0));

			real alpha1 = -ps.beta * C1;
			real alpha2 = -ps.beta * C2;

			real lam1 = (nMassFactors[1] * alpha1 - A12 * alpha2) / det;
			real lam2 = (nMassFactors[0] * alpha2 - A12 * alpha1) / det;

			inc->applyDeltaPos(n * inc->mInv * (lam1 + lam2), inc->IInv * (inCrossFactors[0] * lam1 + inCrossFactors[1] * lam2));
			ref->applyDeltaPos(-n * ref->mInv * (lam1 + lam2), ref->IInv * (-rnCrossFactors[0] * lam1 - rnCrossFactors[1] * lam2));

			return;
		}
	}

	// At this point, the condition number was too high, so resort to the iterative solution.
	
	for (int i = 0; i < ncp; ++i)
	{
		rebuild();
		updateCache();

		ContactPoint& cp = contactPoints[i];
		real C = std::min(cp.penetration + ps.slop, static_cast<real>(0));

		//C = std::clamp(cp.penetration + slop, -slop, static_cast<real>(0));

		//std::cout << C << "\n";

		real dLambda = 0;
		if (nMassFactors[i] != 0)
		{
			dLambda = -ps.beta * C / nMassFactors[i];
		}

		//std::cout << inc->mInv * dLambda << "\n";
		
		inc->applyDeltaPos(n * inc->mInv * dLambda, inc->IInv * inCrossFactors[i] * dLambda);
		ref->applyDeltaPos(-n * ref->mInv * dLambda, ref->IInv * -rnCrossFactors[i] * dLambda);
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

	refEdge = ppOther->refEdge;
	incEdge = ppOther->incEdge;

	contactPoints = std::move(ppOther->contactPoints);
	vRelTarget = std::move(ppOther->vRelTarget);
}

void PolyPolyContact::rebuildPoint(int i)
{
	ContactPoint& cp = contactPoints[i];

	vec2 normal = refEdge->normal();
	vec2 refPoint = refEdge->point1();

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

	vec2 iRelPos, rRelPos;
	for (int i = 0; i < ncp; ++i)
	{
		iRelPos = contactPoints[i].point - inc->position();
		rRelPos = contactPoints[i].point - ref->position();

		inCrossFactors[i] = zcross(iRelPos, n);
		itCrossFactors[i] = zcross(iRelPos, t);
		rnCrossFactors[i] = zcross(rRelPos, n);
		rtCrossFactors[i] = zcross(rRelPos, t);

		nMassFactors[i] = inc->mInv + ref->mInv + inc->IInv * std::pow(inCrossFactors[i], 2) + ref->IInv * std::pow(rnCrossFactors[i], 2);
		tMassFactors[i] = inc->mInv + ref->mInv + inc->IInv * std::pow(itCrossFactors[i], 2) + ref->IInv * std::pow(rtCrossFactors[i], 2);
	}

	if ((ps.simulSolveVel || ps.simulSolvePos) && ncp == 2)
	{
		A12 = inc->mInv + ref->mInv + inc->IInv * inCrossFactors[0] * inCrossFactors[1] + ref->IInv * rnCrossFactors[0] * rnCrossFactors[1];
		det = nMassFactors[0] * nMassFactors[1] - A12 * A12; 
		norm = std::max(nMassFactors[0], nMassFactors[1]) + std::abs(A12);
		real normSquared = norm * norm;

		// Is the condition number less than the threshold?
		wellConditionedVel = normSquared < ps.maxCondVel * det;
		wellConditionedPos = normSquared < ps.maxCondPos * det;

		//std::cout << wellConditionedVel << wellConditionedPos << "\n";
	}
}
