#pragma once

#include "RigidBody.h"
#include "PolyPolyContact.h"
#include "PolyCircleContact.h"
#include "Edge.h"
#include "Vertex.h"
#include "Simplex.h"
#include <unordered_map>
#include <map>

class ConvexPolygon : public RigidBody
{
public:
	ConvexPolygon(const PhysicsSettings& ps, int npoints, real sideLength, real mInv = 0);

	void draw(sf::RenderWindow& window, real fraction, bool debug, sf::Text* text) override;

	std::unique_ptr<ContactConstraint> checkCollision(RigidBody* other) override { return other->checkCollision(this); }
	std::unique_ptr<ContactConstraint> checkCollision(ConvexPolygon* other) override;
	std::unique_ptr<ContactConstraint> checkCollision(Circle* other) override;

	void updateAABB() override;
	void updateFatAABB(real w) override;
	bool pointInside(const vec2& p) const override;

	void onMove() override;

	vec2 edge(int i) const { return edges[i]->global(); }
	vec2 vertex(int i) const { return vertices[i]->global(); }
	vec2 normal(int i) const { return edges[i]->normal(); }

	int nextIndex(int i) const;
	int prevIndex(int i) const;

	std::pair<vec2, Voronoi> closestPoint(const vec2& point);


private:
	void createRegularPolygon(real sideLength);
	void setupRegularPolyMOI(real sideLength);

	void initEdges();
	void initShape();

	const Vertex* supportVertex(const vec2& d) const;

	// Find penetration of other polygon into this polygon along normal of edge e
	std::pair<real, const Vertex*> normalPenetration(const Edge* e, const ConvexPolygon& other) const; // TODO: why mix pointer and reference?

	// Find maximum signed penetration of other polygon into this polygon along any edge normal
	std::tuple<bool, real, const Edge*, const Vertex*> maxSignedPenetration(const ConvexPolygon& other) const;

	real absEdgeDot(const Edge* e, const vec2& d) const;

	const int npoints;
	std::vector<std::unique_ptr<Edge>> edges;
	std::vector<std::unique_ptr<Vertex>> vertices;

	sf::ConvexShape shape;

	std::unordered_map<idType, vec2> separatingAxes;
	
	std::pair<real, real> shadow(const vec2& n);
};