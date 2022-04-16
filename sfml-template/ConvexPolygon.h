#pragma once

#include "RigidBody.h"
#include "PolyPolyContact.h"


// TODO: add Edge & Vertex classes?

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon(int npoints, real sideLength);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override;

	//findContactPoints(const RigidBody* other, int normalIndex,) const;

	//void SAT(const ConvexPolygon* other);

	std::pair<vec2, int> support(const vec2& d) const;

	vec2 transformedPoint(int i) const;
	vec2 transformedEdge(int i) const;
	vec2 transformedNormal(int i) const;

	int nextIndex(int i) const;
	int prevIndex(int i) const;

private:

	void createRegularPolygon(real sideLength);
	void initEdgesAndNormals();
	void initShape();


	// Find penetration of other polygon into this polygon along normal i
	std::pair<real, int> normalPenetration(int i, const ConvexPolygon& other) const;

	// Find maximum signed penetration of other polygon into this polygon along any face normal
	std::tuple<bool, real, int, int> maxSignedPenetration(const ConvexPolygon& other) const;

	real absEdgeDot(int i, const vec2& d);

	bool colliding = false;

	// Points, edges and normals stored in local coordinates
	const int npoints;
	std::vector<vec2> points;
	std::vector<vec2> edges;
	std::vector<vec2> normals;

	sf::ConvexShape shape;
};