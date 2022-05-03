#include "ConvexPolygon.h"


ConvexPolygon::ConvexPolygon(int npoints, real sideLength):
	npoints(npoints)
{
	//omega = 40.0 * pi / 180;
	//vel = { 7, -7 };

	createRegularPolygon(sideLength);
	initEdgesAndNormals();
	initShape();

	moveTo({ 2, 7 });
}

void ConvexPolygon::update(real dt)
{

}

void ConvexPolygon::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateAngle(fraction);

	for (int i = 0; i < npoints; ++i)
	{
		// TODO: function for interpolated vertex coordinates?
		sf::Vector2f pointCoord = sf::Vector2f(rotate(vertices[i].local(), itheta) + ipos) * pixPerUnit;

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
}

std::unique_ptr<ContactConstraint> ConvexPolygon::checkCollision(ConvexPolygon* other)
{
	// Check normal directions of *this
	auto [earlyOutA, penetrationBtoA, normalIndexA, pointIndexB] = this->maxSignedPenetration(*other);

	if (earlyOutA)
	{
		return nullptr;
	}

	// Check normal directions of *other
	auto [earlyOutB, penetrationAtoB, normalIndexB, pointIndexA] = other->maxSignedPenetration(*this);

	if (earlyOutB)
	{
		return nullptr;
	}
	

	ConvexPolygon* ref = nullptr;
	ConvexPolygon* inc = nullptr;

	int incPointIndex = -1;
	int refEdgeIndex = -1;
	
	// TODO: check tolerance and include both relative & absolute
	// Should be considerably less than the slop value, as for persistent contacts 
	// the separation shouldn't exceed the slop
	real tol = 1e-4; // 0.01;
	if (penetrationBtoA > penetrationAtoB + tol)
	{
		// Penetrations are signed, so here A penetrates into B more than B penetrates into A
		// i.e. the minimum translation vector points from A to B
		ref = this;
		inc = other;

		refEdgeIndex = normalIndexA;
		incPointIndex = pointIndexB;
	}
	else
	{
		ref = other;
		inc = this;

		refEdgeIndex = normalIndexB;
		incPointIndex = pointIndexA;
	}

	// The deepest point has index incPointIndex, which is part of both the edge with index incPointIndex
	// and the previous edge. The incident edge is least well-aligned with the reference normal.
	vec2 normal = ref->edges[refEdgeIndex].normal();
	int incEdgeIndex = incPointIndex;
	int alternativeEdgeIndex = inc->prevIndex(incPointIndex);
	
	// TODO: add tolerance?
	if (inc->absEdgeDot(alternativeEdgeIndex, normal) < inc->absEdgeDot(incPointIndex, normal))
	{
		incEdgeIndex = alternativeEdgeIndex;
	}
	
	return std::make_unique<PolyPolyContact>(ref, inc, refEdgeIndex, incEdgeIndex);
}

void ConvexPolygon::onMove()
{
	for (int i = 0; i < npoints; ++i)
	{
		vertices[i].recompute(position(), angle());
	}
}

void ConvexPolygon::onRotate()
{
	for (int i = 0; i < npoints; ++i)
	{
		vertices[i].recompute(position(), angle());
		edges[i].recompute(angle());
	}
}


// Returns <signed penetration, index of deepest point> 
std::pair<real, int> ConvexPolygon::normalPenetration(int i, const ConvexPolygon& other) const
{
	vec2 normal = edges[i].normal();

	Vertex supportPointThis = vertices[i];
	Vertex supportPointOther = other.support(-normal);
	real signedDistance = dot(supportPointOther.global() - supportPointThis.global(), normal);

	return {signedDistance, supportPointOther.index()};
}

// Returns <early out, max signed penetration, index of normal, index of deepest point> 
// If the first return value is true, should discard the others
std::tuple<bool, real, int, int> ConvexPolygon::maxSignedPenetration(const ConvexPolygon& other) const
{
	bool earlyOut = false;
	real maxPenetration = std::numeric_limits<real>::lowest();
	int normalIndex = -1;
	int pointIndex = -1;

	for (int i = 0; i < npoints; ++i)
	{
		auto [penetration, pindex] = normalPenetration(i, other);

		if (penetration > 0)
		{
			earlyOut = true;
			break;
		}

		if (penetration > maxPenetration)
		{
			maxPenetration = penetration;
			pointIndex = pindex;
			normalIndex = i;
		}
	}

	return { earlyOut, maxPenetration, normalIndex, pointIndex };
}

real ConvexPolygon::absEdgeDot(int i, const vec2& d) const
{
	return std::abs(dot(edges[i].global(), d));
}


int ConvexPolygon::nextIndex(int i) const
{
	return (i + 1 == npoints) ? 0 : i + 1;
}

int ConvexPolygon::prevIndex(int i) const
{
	return (i == 0) ? npoints - 1 : i - 1;
}


Vertex ConvexPolygon::support(const vec2& d) const
{
	real largestDot = std::numeric_limits<real>::lowest();

	int index = -1;
	for (int i = 0; i < npoints; ++i)
	{
		real dotProduct = dot(vertices[i].global(), d);

		if (dotProduct > largestDot)
		{
			largestDot = dotProduct;
			index = i;
		}
	}

	return vertices[index];
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

		vertices.emplace_back(i, point, *this);
	}
}

void ConvexPolygon::initEdgesAndNormals()
{
	assert(edges.empty());
	
	for (int i = 0; i < npoints; ++i)
	{
		vec2 edge = vertices[nextIndex(i)].local() - vertices[i].local();
		edges.emplace_back(i, edge, *this);
	}
}

void ConvexPolygon::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);

	shape.setPointCount(npoints);
}
