#include "ConvexPolygon.h"


ConvexPolygon::ConvexPolygon(int npoints, real sideLength):
	npoints(npoints)
{
	moveTo({ 2, 7 });


	//omega = 40.0 * pi / 180;
	//vel = { 7, -7 };

	createRegularPolygon(sideLength);
	initEdgesAndNormals();
	initShape();
}

void ConvexPolygon::update(real dt)
{

}

void ConvexPolygon::draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr)
{
	vec2 ipos = interpolatePos(fraction);
	real itheta = interpolateTheta(fraction);

	for (int i = 0; i < npoints; ++i)
	{
		sf::Vector2f pointCoord = sf::Vector2f(transform(points[i], ipos, itheta) * pixPerUnit);

		shape.setPoint(i, pointCoord);
	
		if (debug && text)
		{
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
	real tol = 0.01;
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
	vec2 normal = ref->transformedNormal(refEdgeIndex);
	int incEdgeIndex = incPointIndex;
	int alternativeEdgeIndex = inc->prevIndex(incPointIndex);
	
	// TODO: add tolerance?
	if (inc->absEdgeDot(alternativeEdgeIndex, normal) < inc->absEdgeDot(incPointIndex, normal))
	{
		incEdgeIndex = alternativeEdgeIndex;
	}

	//std::cout << "ref: " << refEdgeIndex << " | inc: " << incEdgeIndex << '\n';
	
	return std::make_unique<PolyPolyContact>(ref, inc, refEdgeIndex, incEdgeIndex);
}


// Returns <signed penetration, index of deepest point> 
std::pair<real, int> ConvexPolygon::normalPenetration(int i, const ConvexPolygon& other) const
{
	vec2 normal = transformedNormal(i);
	vec2 supportPointThis = transformedPoint(i);
	auto [supportPointOther, supportIndexOther] = other.support(-normal);
	real signedDistance = dot(supportPointOther - supportPointThis, normal);

	return {signedDistance, supportIndexOther};
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

real ConvexPolygon::absEdgeDot(int i, const vec2& d)
{
	return std::abs(dot(transformedEdge(i), d));
}


int ConvexPolygon::nextIndex(int i) const
{
	return (i + 1 == npoints) ? 0 : i + 1;
}

int ConvexPolygon::prevIndex(int i) const
{
	return (i == 0) ? npoints - 1 : i - 1;
}


// Return <support point, index>
std::pair<vec2, int> ConvexPolygon::support(const vec2& d) const
{
	real largestDot = std::numeric_limits<real>::lowest();

	vec2 supportPoint;
	int index = -1;
	
	for (int i = 0; i < npoints; ++i)
	{
		vec2 point = transformedPoint(i);
		real dotProduct = dot(point, d);

		if (dotProduct > largestDot)
		{
			largestDot = dotProduct;
			supportPoint = point;
			index = i;
		}
	}

	return { supportPoint, index };
}

vec2 ConvexPolygon::transformedPoint(int i) const
{
	return transform(points[i], pos, theta);
}

vec2 ConvexPolygon::transformedEdge(int i) const
{
	return rotate(edges[i], theta);
}

vec2 ConvexPolygon::transformedNormal(int i) const
{
	return rotate(normals[i], theta);
}

void ConvexPolygon::createRegularPolygon(real sideLength)
{
	// Shouldn't be calling more than once per polygon!
	assert(points.empty());
		
	real theta = 2 * pi / npoints;
	real r = sideLength / (2 * std::sin(theta / 2));
	

	for (int i = 0; i < npoints; ++i)
	{
		real angle = i * theta;

		// Ensure bottom edge is horizontal
		// Bottom-right corner makes angle (90 - theta/2) deg with horizontal
		angle += pi / 2 - theta / 2;

		vec2 pos = { r * std::cos(angle), r * std::sin(angle) };

		points.push_back(pos);
	}
}

void ConvexPolygon::initEdgesAndNormals()
{
	assert(edges.empty());
	assert(normals.empty());

	for (int i = 0; i < npoints; ++i)
	{
		vec2 edge = points[nextIndex(i)] - points[i];

		edges.push_back(edge);

		// TODO: Verify that the normal points outwards
		normals.push_back(perp(normalise(edge)));
	}
}

void ConvexPolygon::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-2);

	shape.setPointCount(npoints);
}
