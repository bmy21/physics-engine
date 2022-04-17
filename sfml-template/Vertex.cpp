#include "Vertex.h"

Vertex::Vertex(int i, const vec2& v, const vec2& pos, real angle):
	i(i),
	localCoords(v)
{
	recompute(pos, angle);
}

Vertex::Vertex(int i, const vec2& v, const RigidBody& rb):
	Vertex(i, v, rb.position(), rb.angle())
{
}

void Vertex::recompute(const vec2& pos, real theta)
{
	globalCoords = transform(localCoords, pos, theta);
}
