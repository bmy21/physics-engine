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

bool ConvexPolygon::overlaps(const ConvexPolygon* other) const
{
	auto [thisEarlyOut, thisPenetration, thisNormalIndex, thisPointIndex] = this->maxSignedPenetration(*other);
	
	if (thisEarlyOut)
	{
		return false;
	}

	auto [otherEarlyOut, otherPenetration, otherNormalIndex, otherPointIndex] = other->maxSignedPenetration(*this);

	if (otherEarlyOut)
	{
		return false;
	}
	

	bool thisIsReference = false;
	
	// TODO: check tolerance and include both relative & absolute
	real tol = 0.01;
	if (thisPenetration > otherPenetration + tol)
	{
		thisIsReference = true;
		std::cout << "this\n";
	}
	else
	{
		thisIsReference = false;
		std::cout << "other\n";
	}
	
	return true;
}


// Returns <signed penetration, index of deepest point> 
std::pair<real, int> ConvexPolygon::normalPenetration(int i, const ConvexPolygon& other) const
{
	vec2 normal = transformedNormal(i);
	vec2 supportThis = transformedPoint(i);
	auto [supportPointOther, supportIndexOther] = other.support(-normal);
	real signedDistance = dot(supportPointOther - supportThis, normal) / magnitude(normal);

	return std::make_pair(signedDistance, supportIndexOther);
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

		if (penetration > maxPenetration)
		{
			maxPenetration = penetration;
			pointIndex = pindex;
			normalIndex = i;
		}

		if (penetration > 0)
		{
			earlyOut = true;
			break;
		}
	}

	return std::make_tuple(earlyOut, maxPenetration, normalIndex, pointIndex);
}

//void ConvexPolygon::SAT(const ConvexPolygon* other)
//{
//	bool overlaps = true;
//
//	bool thisIsReference = false;
//
//	real maxSignedPenetrationThis = std::numeric_limits<real>::lowest();
//	int axisMinPenetrationThis = -1;
//
//	for (int i = 0; i < npoints; ++i)
//	{
//		real signedPenetration = normalPenetration(i, *other);
//
//		if (signedPenetration > maxSignedPenetrationThis)
//		{
//			maxSignedPenetrationThis = signedPenetration;
//			axisMinPenetrationThis = i;
//		}
//
//		if (signedPenetration > 0)
//		{
//			overlaps = false;
//		}
//	}
//
//	for (int i = 0; i < other->npoints; ++i)
//	{
//		real signedPenetration = other->normalPenetration(i, *this);
//
//		if (signedPenetration > 0)
//		{
//			overlaps = false;
//		}
//	}
//
//}

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

	return std::make_pair(supportPoint, index);
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
		int next = (i + 1 == npoints) ? 0 : i + 1;
		vec2 edge = points[next] - points[i];

		edges.push_back(edge);

		// TODO: Verify that the normal points outwards
		normals.push_back(perp(edge));
	}
	
}

void ConvexPolygon::initShape()
{
	shape.setFillColor(sf::Color::Transparent);
	shape.setOutlineColor(sf::Color::Black);
	shape.setOutlineThickness(-1);

	shape.setPointCount(npoints);
}
