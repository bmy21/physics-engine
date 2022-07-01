#include "Edge.h"

Edge::Edge(int i, const vec2& edge, real angle):
	i(i), localEdge(edge)
{
	localNorm = perp(normalise(localEdge));
	recompute(angle);
}

Edge::Edge(int i, const vec2& edge, const RigidBody& rb):
	Edge(i, edge, rb.angle())
{

}

void Edge::recompute(real angle)
{
	globalEdge = rotate(localEdge, angle);
	globalNorm = rotate(localNorm, angle);
}

void Edge::recompute(real c, real s)
{
	globalEdge = rotate(localEdge, c, s);
	globalNorm = rotate(localNorm, c, s);
}
