#pragma once

#include "RigidBody.h"
#include "PolyPolyContact.h"
#include "Edge.h"
#include "Vertex.h"

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon(int npoints, real sideLength);

	void update(real dt) override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override;

	bool pointInside(const vec2& p) override;

	void onMove() override;
	void onRotate() override;

	Vertex support(const vec2& d) const;

	vec2 edge(int i) const { return edges[i].global(); }
	vec2 vertex(int i) const { return vertices[i].global(); }
	vec2 normal(int i) const { return edges[i].normal(); }

	int nextIndex(int i) const;
	int prevIndex(int i) const;


private:

	void createRegularPolygon(real sideLength);
	void initEdgesAndNormals();
	void initShape();


	// Find penetration of other polygon into this polygon along normal i
	std::pair<real, int> normalPenetration(int i, const ConvexPolygon& other) const;

	// Find maximum signed penetration of other polygon into this polygon along any edge normal
	std::tuple<bool, real, int, int> maxSignedPenetration(const ConvexPolygon& other) const;

	real absEdgeDot(int i, const vec2& d) const;

	const int npoints;
	std::vector<Edge> edges;
	std::vector<Vertex> vertices;

	sf::ConvexShape shape;
};