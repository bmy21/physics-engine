#pragma once

#include "Vertex.h"

class Simplex
{
public:
	std::tuple<vec2, vec2, Voronoi> closestPoint(const vec2& point);
	
	bool contains(const Vertex* vertex) const;
	void addVertex(const Vertex* vertex);
	void cleanupVertices();

private:
	class SimplexVertex;
	std::vector<SimplexVertex> vertices;
};


class Simplex::SimplexVertex
{
public:
	SimplexVertex(const Vertex* v) : vertex(v) { };
	vec2 coords() const { return vertex->global(); }

	void markForRemoval() { remove = true; }
	void unsetRemoveFlag() { remove = false; }
	bool removeFlagSet() const { return remove; }
	bool matches(const Vertex* v) const { return v == vertex; }

private:
	const Vertex* vertex = nullptr;
	bool remove = false;
};