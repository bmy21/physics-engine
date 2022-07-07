#pragma once
#include "Utils.h"
#include "RigidBody.h"
#include "Vertex.h"

class Edge
{
public:
	Edge(int i, const vec2& edge, real angle);
	Edge(int i, const vec2& edge, const RigidBody& rb);

	vec2 local() const { return localEdge; }
	vec2 global() const { return globalEdge; }
	vec2 normal() const { return globalNorm; }

	vec2 point1() const { return mv1->global(); }
	vec2 point2() const { return mv2->global(); }

	int v1index() const { return mv1->index(); }
	int v2index() const { return mv2->index(); }
	int index() const { return i; }

	const Edge* prev() const { return mPrev; }
	const Edge* next() const { return mNext; }
	const Vertex* v1() const { return mv1; }
	const Vertex* v2() const { return mv2; }

	void linkPrev(const Edge* e) { mPrev = e; }
	void linkNext(const Edge* e) { mNext = e; }
	void linkv1(const Vertex* v) { mv1 = v; }
	void linkv2(const Vertex* v) { mv2 = v; }

	// Recalculate the global vector and global normal
	void recompute(real angle);
	void recompute(real c, real s);

private:
	vec2 localEdge;
	vec2 globalEdge;

	vec2 localNorm;
	vec2 globalNorm;

	const Edge* mPrev = nullptr;
	const Edge* mNext = nullptr;
	const Vertex* mv1 = nullptr;
	const Vertex* mv2 = nullptr;

	// this Edge points from Vertex i to Vertex i + 1
	const int i;
};

