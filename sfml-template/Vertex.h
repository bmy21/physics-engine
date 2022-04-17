#pragma once
#include "Utils.h"
#include "RigidBody.h"

class Vertex
{
public:
	Vertex(int i, const vec2& v, const vec2& pos, real angle);
	Vertex(int i, const vec2& v, const RigidBody& rb);

	vec2 local() const { return localCoords; }
	vec2 global() const { return globalCoords; }
	int index() const { return i; }

	// Recalculate the global coords
	void recompute(const vec2& pos, real theta);

private:
	int i = -1;
	vec2 localCoords;
	vec2 globalCoords;
};

