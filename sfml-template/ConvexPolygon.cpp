#include "ConvexPolygon.h"
#include "Circle.h"

ConvexPolygon::ConvexPolygon(const PhysicsSettings& ps, int npoints, real sideLength, real mInv):
	npoints(npoints),
	RigidBody(ps, mInv)
{
	createRegularPolygon(sideLength);
	setupRegularPolyMOI(sideLength);

	initEdges();
	initShape();
}

void ConvexPolygon::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateAngle(fraction);

	for (int i = 0; i < npoints; ++i)
	{
		sf::Vector2f pointCoord = vertices[i]->global(ipos, itheta) * pixPerUnit;
		shape.setPoint(i, pointCoord);
	
		if (debug && text)
		{
			text->setCharacterSize(30);
			text->setFillColor(sf::Color::Blue);
			text->setString(std::to_string(i));
			text->setPosition(pointCoord);
			centre(*text);

			window.draw(*text);
		}
	}

	window.draw(shape);

	real rad = 5;
	sf::CircleShape circle(rad);
	circle.setOrigin(rad, rad);
	circle.setFillColor(sf::Color::Magenta);

	//vec2 closest = closestPoint({ 0, 0 }).first;
	//circle.setPosition(closest.x * pixPerUnit, closest.y * pixPerUnit);
	//window.draw(circle);
}

std::unique_ptr<ContactConstraint> ConvexPolygon::checkCollision(ConvexPolygon* other)
{
	//TODO: allow processing time at the start for all separating vectors to be cached?


	// Quickly rule out collisions using an AABB test
	if (!aabb.overlaps(other->aabb))
	{
		return nullptr;
	}


	//vec2 sepAxis = separatingAxes[other->id];

	//if (!isZero(sepAxis) && !rangeOverlaps(shadow(sepAxis), other->shadow(sepAxis)))
	//{
	//	return nullptr;
	//}

	// Check normal directions of *this
	auto [earlyOutA, penetrationBtoA, edgeA, vertexB] = this->maxSignedPenetration(*other);

	if (earlyOutA)
	{
		//separatingAxes[other->id] = edgeA->normal();
		return nullptr;
	}

	// Check normal directions of *other
	auto [earlyOutB, penetrationAtoB, edgeB, vertexA] = other->maxSignedPenetration(*this);

	if (earlyOutB)
	{
		//separatingAxes[other->id] = edgeB->normal();
		return nullptr;
	}
	
	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;
	const Edge* refEdge = nullptr;
	const Edge* incEdge = nullptr;
	const Vertex* incVertex = nullptr;
	
	// TODO: include both relative & absolute tolerance?
	if (penetrationBtoA > penetrationAtoB + ps.refEdgeAbsTol)
	{
		// Penetrations are signed, so here A penetrates into B more than B penetrates into A
		// i.e. the minimum translation vector points from A to B
		ref = this;
		inc = other;

		refEdge = edgeA;
		incVertex = vertexB;
	}
	else
	{
		ref = other;
		inc = this;

		refEdge = edgeB;
		incVertex = vertexA;
	}

	// The deepest point has index incPointIndex, which is part of both the edge with index incPointIndex
	// and the previous edge. The incident edge is least well-aligned with the reference normal.
	vec2 normal = refEdge->normal();
	
	// TODO: add tolerance?
	// TODO: use normal dot products?
	if (inc->absEdgeDot(incVertex->e1(), normal) < inc->absEdgeDot(incVertex->e2(), normal))
	{
		incEdge = incVertex->e1();
	}
	else
	{
		incEdge = incVertex->e2();
	}
	
	return std::make_unique<PolyPolyContact>(ref, inc, refEdge, incEdge, ps);
}

std::unique_ptr<ContactConstraint> ConvexPolygon::checkCollision(Circle* other)
{
	vec2 centre = other->position();
	real rad = other->radius();

	auto [closest, region] = closestPoint(centre);
	
	if (region == Voronoi::Inside)
	{
		// Deep contact
		// Normal points from polygon to circle
		real maxSignedDistance = std::numeric_limits<real>::lowest();
		vec2 projection, n;

		for (auto& e : edges)
		{
			real signedDistance = dot(closest - e->point1(), e->normal());

			if (signedDistance > maxSignedDistance)
			{
				maxSignedDistance = signedDistance;
				n = e->normal();
				projection = closest - signedDistance * n;
			}
		}

		vec2 localNormal = vecToLocal(n);
		vec2 localClosest = pointToLocal(projection);

		return std::make_unique<PolyCircleContact>(this, other, localNormal, localClosest, region, ps);
	}
	else
	{
		if (magnitude(centre - closest) < rad)
		{
			// Shallow contact
			// Normal points from polygon to circle
			vec2 localClosest = pointToLocal(closest);
			
			vec2 localNormal = {0, 0};

			if (region == Voronoi::Edge)
			{
				vec2 n = centre - closest;

				if (!isZero(n))
				{
					n = normalise(n);
				}

				localNormal = vecToLocal(n);
			}

			return std::make_unique<PolyCircleContact>(this, other, localNormal, localClosest, region, ps);
		}
		else
		{
			// No contact
			return nullptr;
		}
	}
}

void ConvexPolygon::updateAABB()
{
	std::tie(aabb.left, aabb.right) = shadow({ 1, 0 });
	std::tie(aabb.top, aabb.bottom) = shadow({ 0, 1 });
}

bool ConvexPolygon::pointInside(const vec2& p) const
{
	for (auto& e : edges)
	{
		if (dot(p - e->point1(), e->normal()) > 0)
		{
			return false;
		}
	}

	return true;
}

void ConvexPolygon::onMove()
{
	for (auto& v : vertices)
	{
		v->recompute(position(), angle());
	}
}

void ConvexPolygon::onRotate()
{
	for (auto& v : vertices)
	{
		v->recompute(position(), angle());
	}

	for (auto& e : edges)
	{
		e->recompute(angle());
	}
}


// Returns <signed penetration, deepest vertex> 
std::pair<real, const Vertex*> ConvexPolygon::normalPenetration(const Edge* e, const ConvexPolygon& other) const
{
	vec2 normal = e->normal();

	const Vertex* supportPointOther = other.supportVertex(-normal);
	real signedDistance = dot(supportPointOther->global() - e->point1(), normal);

	return {signedDistance, supportPointOther};
}

// Returns <early out, max signed penetration, edge of max signed penetration, penetrating vertex> 
// If the first return value is true, should discard the others
std::tuple<bool, real, const Edge*, const Vertex*> ConvexPolygon::maxSignedPenetration(const ConvexPolygon& other) const
{
	bool earlyOut = false;
	real maxPenetration = std::numeric_limits<real>::lowest();

	const Edge* edge = nullptr;
	const Vertex* vertex = nullptr;

	for (auto& e : edges)
	{
		auto [penetration, v] = normalPenetration(e.get(), other);

		if (penetration > 0)
		{
			earlyOut = true;
			edge = e.get();
			break;
		}

		if (penetration > maxPenetration)
		{
			maxPenetration = penetration;
			vertex = v;
			edge = e.get();
		}
	}

	return { earlyOut, maxPenetration, edge, vertex };
}

real ConvexPolygon::absEdgeDot(const Edge* e, const vec2& d) const
{
	return std::abs(dot(e->global(), d));
}

std::pair<real, real> ConvexPolygon::shadow(const vec2& n)
{
	// Assumes n is normalised

	real smallest = std::numeric_limits<real>::max();
	real largest = std::numeric_limits<real>::lowest();
	
	for (const auto& v : vertices)
	{
		real p = dot(v->global(), n);

		if (p > largest)
		{
			largest = p;
		}
		if (p < smallest)
		{
			smallest = p;
		}
	}

	return { smallest, largest };
}

int ConvexPolygon::nextIndex(int i) const
{
	return (i + 1 == npoints) ? 0 : i + 1;
}

int ConvexPolygon::prevIndex(int i) const
{
	return (i == 0) ? npoints - 1 : i - 1;
}

// Returns <closest point, region type>
std::pair<vec2, Voronoi> ConvexPolygon::closestPoint(const vec2& point)
{
	// TODO: caching of previous result?
	int nIter = 0;
	
	Simplex s;
	s.addVertex(vertices[0].get());
	
	while (true)
	{
		auto [closest, d, region] = s.closestPoint(point);
		
		// Remove non-contributing vertices
		s.cleanupVertices();

		if (region == Voronoi::Inside)
		{
			// NOTE: previously first return value was "point" 
			return { closest, region };
		}

		const Vertex* newSupport = supportVertex(d);

		if (s.contains(newSupport) || isZero(d))
		{
			return { closest, region };
		}

		s.addVertex(newSupport); 

		// Terminate after a set number of iterations
		if (++nIter >= ps.maxIterGJK)
		{
			//std::cout << "max GJK iterations exceeded\n";
			return { closest, region };
		}
	}
}

const Vertex* ConvexPolygon::supportVertex(const vec2& d) const
{
	real largestDot = std::numeric_limits<real>::lowest();

	Vertex* vertex = nullptr;
	for (auto& v : vertices)
	{
		real dotProduct = dot(v->global(), d);

		if (dotProduct > largestDot)
		{
			largestDot = dotProduct;
			vertex = v.get();
		}
	}

	return vertex;
}

void ConvexPolygon::createRegularPolygon(real sideLength)
{
	assert(vertices.empty());

	real centralAngle = 2 * pi / npoints;
	real r = sideLength / (2 * std::sin(centralAngle / 2));

	for (int i = 0; i < npoints; ++i)
	{
		real pointAngle = i * centralAngle;

		// Ensure bottom edge is horizontal
		// Bottom-right corner makes angle (90 - theta/2) deg with horizontal
		pointAngle += pi / 2 - centralAngle / 2;

		vec2 point = { r * std::cos(pointAngle), r * std::sin(pointAngle) };

		vertices.emplace_back(std::make_unique<Vertex>(i, point, *this));
	}
}

void ConvexPolygon::setupRegularPolyMOI(real sideLength)
{
	real preFactor = mInv * 24 / (sideLength * sideLength);
	real cot = 1 / std::tan(pi / npoints);
	real trigFactor = 1 + 3 * cot * cot;

	IInv = preFactor / trigFactor;
}

void ConvexPolygon::initEdges()
{
	assert(edges.empty());
	
	// First populate the edge vector
	for (int i = 0; i < npoints; ++i)
	{
		vec2 localEdge = vertices[nextIndex(i)]->local() - vertices[i]->local();
		edges.emplace_back(std::make_unique<Edge>(i, localEdge, *this));
	}

	// Now link the edges & vertices together
	for (int i = 0; i < npoints; ++i)
	{
		int iPrev = prevIndex(i), iNext = nextIndex(i);

		edges[i]->linkPrev(edges[iPrev].get());
		edges[i]->linkNext(edges[iNext].get());

		edges[i]->linkv1(vertices[i].get());
		edges[i]->linkv2(vertices[iNext].get());

		vertices[i]->linke1(edges[iPrev].get());
		vertices[i]->linke2(edges[i].get());
	}
}

void ConvexPolygon::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);

	shape.setPointCount(npoints);
}
