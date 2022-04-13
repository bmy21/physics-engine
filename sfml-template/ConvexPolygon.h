#pragma once

#include "RigidBody.h"

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon(int npoints, real sideLength);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

	bool overlaps(const RigidBody* other) const override { return other->overlaps(this); } 
	bool overlaps(const ConvexPolygon* other) const override;

	

	//void SAT(const ConvexPolygon* other);

	std::pair<vec2, int> support(const vec2& d) const;

	vec2 transformedPoint(int i) const;
	vec2 transformedEdge(int i) const;
	vec2 transformedNormal(int i) const;

private:

	void createRegularPolygon(real sideLength);
	void initEdgesAndNormals();
	void initShape();

	// Find penetration of other polygon into this polygon along normal i
	std::pair<real, int> normalPenetration(int i, const ConvexPolygon& other) const;
	std::tuple<bool, real, int, int> maxSignedPenetration(const ConvexPolygon& other) const;

	//bool SAT(bool* thisIsRef, int* normalIndex, int* pointIndex, const ConvexPolygon& other) const;

	bool colliding = false;

	// Points, edges and normals stored in local coordinates
	const int npoints;
	std::vector<vec2> points;
	std::vector<vec2> edges;
	std::vector<vec2> normals;

	sf::ConvexShape shape;
};