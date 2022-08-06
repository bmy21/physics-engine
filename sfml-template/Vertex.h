#pragma once
#include "Utils.h"
#include "RigidBody.h"

class Edge;

class Vertex
{
public:
	Vertex(int i, const vec2& v, const vec2& pos, real angle);
	Vertex(int i, const vec2& v, const RigidBody& rb);

	vec2 local() const { return localCoords; }
	vec2 global() const { return globalCoords; }
	vec2 global(const vec2& pos, real angle) const { return transform(localCoords, pos, angle); }
	int index() const { return i; }

	const Edge* e1() const { return me1; }
	const Edge* e2() const { return me2; }

	void linke1(const Edge* e) { me1 = e; }
	void linke2(const Edge* e) { me2 = e; }

	void changeLocal(const vec2& v, const RigidBody& rb);

	// Recalculate the global coords
	void recompute(const vec2& pos, real theta);
	void recompute(const vec2& pos, real c, real s);

private:
	const int i;

	vec2 localCoords;
	vec2 globalCoords;

	const Edge* me1 = nullptr;
	const Edge* me2 = nullptr;
};

