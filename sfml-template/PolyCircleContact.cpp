#include "PolyCircleContact.h"

PolyCircleContact::PolyCircleContact(ConvexPolygon* p, Circle* c, 
	const vec2& localNormal, const vec2& localRefPoint, Voronoi region, const PhysicsSettings& ps):
	p(p), c(c), localNormal(localNormal), localRefPoint(localRefPoint), region(region),
	ContactConstraint(ps, p, c)
{
	// NOTE: warm starting the rolling friction can cause infinite spinning
	setRollingFriction();
}

void PolyCircleContact::initPoints()
{
	updateNormal();
	contactPoints.resize(1);
	rebuildPoint(contactPoints.front());
}

void PolyCircleContact::rebuildPoint(ContactPoint& cp)
{
	vec2 globalRefPoint = p->pointToGlobal(localRefPoint);
	vec2 circlePoint = c->furthestPoint(-n);

	cp.penetration = dot(circlePoint - globalRefPoint, n);
	cp.point = circlePoint - cp.penetration * n;
}

void PolyCircleContact::updateNormal()
{
	if (region == Voronoi::Vertex)
	{
		n = c->position() - p->pointToGlobal(localRefPoint);
		if (!isZero(n))
		{
			n = normalise(n);
		}
	}
	else 
	{
		// Edge or Inside
		n = p->vecToGlobal(localNormal);
	}
}

void PolyCircleContact::setRollingFriction()
{
	// Only enable rolling friction if the circle is in contact with an edge
	if (region == Voronoi::Vertex)
	{
		disableRollingFriction();
	}
	else
	{
		enableRollingFriction();
	}
}
