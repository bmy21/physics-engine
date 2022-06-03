#pragma once
#include "Utils.h"
#include "RigidBody.h"

class Vertex;

class Edge
{
public:
	Edge(int i, const vec2& edge, real angle);
	Edge(int i, const vec2& edge, const RigidBody& rb);

	vec2 local() const { return localEdge; }
	vec2 global() const { return globalEdge; }
	vec2 normal() const { return globalNorm; }

	int index() const { return i; }

	// Recalculate the global vector and global normal
	void recompute(real angle);

	// TODO: pointers to previous & next edges
	const Edge* prev = nullptr;
	const Edge* next = nullptr;
	const Vertex* v1 = nullptr;
	const Vertex* v2 = nullptr;


private:
	vec2 localEdge;
	vec2 globalEdge;

	vec2 localNorm;
	vec2 globalNorm;

	// this Edge points from Vertex i to Vertex i + 1
	int i = -1;
	
};

