#include "ConvexPolygon.h"
#include "Circle.h"

ConvexPolygon::ConvexPolygon(const PhysicsSettings& ps, int npoints, real sideLength, real mInv):
	npoints(npoints),
	RigidBody(ps, mInv)
{
	createRegularPolygon(sideLength);

	initialise();
}

ConvexPolygon::ConvexPolygon(const PhysicsSettings& ps, const std::vector<vec2>& points, real mInv):
	npoints(points.size()),
	RigidBody(ps, mInv)
{
	for (int i = 0; i < npoints; ++i)
	{
		vertices.emplace_back(std::make_unique<Vertex>(i, points[i], *this));
	}

	initialise();
}

void ConvexPolygon::draw(sf::RenderWindow& window, real fraction, bool debug, sf::Text* text)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateAngle(fraction);

	for (int i = 0; i < npoints; ++i)
	{
		vec2 v = vertices[i]->global(ipos, itheta) * ps.pixPerUnit;
		sf::Vector2f pointCoord(v.x, v.y); 

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
}

std::unique_ptr<ContactConstraint> ConvexPolygon::checkCollision(ConvexPolygon* other)
{
	// Quickly rule out collisions using an AABB test
	if (!aabb.overlaps(other->aabb))
	{
		return nullptr;
	}

	// Check normal directions of *this
	auto [earlyOutA, penetrationBtoA, edgeA, vertexB] = this->maxSignedPenetration(*other);

	if (earlyOutA)
	{
		return nullptr;
	}

	// Check normal directions of *other
	auto [earlyOutB, penetrationAtoB, edgeB, vertexA] = other->maxSignedPenetration(*this);

	if (earlyOutB)
	{
		return nullptr;
	}
	
	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;
	const Edge* refEdge = nullptr;
	const Edge* incEdge = nullptr;
	const Vertex* incVertex = nullptr;
	
	// TODO: could include both relative & absolute tolerance here?
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
	// Quickly rule out collisions using an AABB test
	if (!aabb.overlaps(other->getAABB()))
	{
		return nullptr;
	}

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
	std::tie(aabb.lower.x, aabb.upper.x) = shadow({ 1, 0 });
	std::tie(aabb.lower.y, aabb.upper.y) = shadow({ 0, 1 });
}

void ConvexPolygon::updateFatAABB()
{
	std::tie(aabbFat.lower.x, aabbFat.upper.x) = shadow({ 1, 0 });
	std::tie(aabbFat.lower.y, aabbFat.upper.y) = shadow({ 0, 1 });

	aabbFat.lower.x -= ps.aabbFattening;
	aabbFat.lower.y -= ps.aabbFattening;
	aabbFat.upper.x += ps.aabbFattening;
	aabbFat.upper.y += ps.aabbFattening;
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

void ConvexPolygon::centreOnCOM()
{
	// Adjust local vertices so the centre of mass is at the origin

	vec2 cm = calculateCOM();

	for (auto& v : vertices)
	{
		v->changeLocal(v->local() - cm, *this);
	}

	// Coordinates of the original origin in the new coordinate system
	setRefPoint(-cm);
}

vec2 ConvexPolygon::calculateCOM() const
{
	vec2 ref = vertices.front()->local();

	real area = 0;
	vec2 numerator;

	for (int i = 1; i < npoints - 1; ++i)
	{
		vec2 u = vertices[i]->local() - ref;
		vec2 v = vertices[i + 1]->local() - ref;

		real dA = 0.5 * std::abs(zcross(u, v));
		vec2 dCM = (u + v) * static_cast<real>(1. / 3.);

		area += dA;
		numerator += dCM * dA;
	}

	// Centroid relative to ref point
	vec2 cm = numerator / area;

	// Transform centroid back to local origin
	cm += ref;

	return cm;
}

real ConvexPolygon::calculateInvMOI() const
{
	real area = 0;
	real I = 0;

	for (int i = 0; i < npoints ; ++i)
	{
		vec2 u = vertices[i]->local();
		vec2 v = vertices[nextIndex(i)]->local();

		real crossFactor = std::abs(zcross(u, v));
		real dotFactor = dot(u, u) + dot(u, v) + dot(v, v);

		real dA = 0.5 * crossFactor;
		real dI = (crossFactor * dotFactor) / static_cast<real>(12);

		area += dA;
		I += dI;
	}

	real invDensity = area * mInv();

	return invDensity / I;
}

void ConvexPolygon::onMove()
{
	real c = std::cos(angle()), s = std::sin(angle());

	std::for_each(std::execution::unseq, vertices.begin(), vertices.end(), [&](const std::unique_ptr<Vertex>& v)
	{
		v->recompute(position(), c, s);
	});

	std::for_each(std::execution::unseq, edges.begin(), edges.end(), [&](const std::unique_ptr<Edge>& e)
	{
		e->recompute(c, s);
	});
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
	// NOTE: perhaps worth caching the previous result?
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

		vec2 point = { r * std::cos(pointAngle), r * std::sin(pointAngle)};

		vertices.emplace_back(std::make_unique<Vertex>(i, point, *this));
	}
}

void ConvexPolygon::setupRegularPolyMOI(real sideLength)
{
	real preFactor = mInv() * 24 / (sideLength * sideLength);
	real cot = 1 / std::tan(pi / npoints);
	real trigFactor = 1 + 3 * cot * cot;

	setIInv(preFactor / trigFactor);
}

void ConvexPolygon::initialise()
{
	// Common setup after Vertex coordinates have been established
	centreOnCOM();
	setIInv(calculateInvMOI());
	initEdges();
	initShape();
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
	sf::Color col(196, 250, 248);

	//col = sf::Color::Transparent;
	//col = sf::Color::Black;

	shape.setFillColor(col);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);

	shape.setPointCount(npoints);
}
