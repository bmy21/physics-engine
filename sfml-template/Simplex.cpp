#include "Simplex.h"

// Returns <closest point, new search direction, region type>
std::tuple<vec2, vec2, Voronoi> Simplex::closestPoint(const vec2& point)
{
    // TODO: reduce code duplication (search direction calculation)
    
    // Note: the sign of a 2-point barycentric coordinate doesn't depend on whether the division
    // has taken place, but for a 3-point coordinate it does.

    int npoints = vertices.size();

    vec2 pA = vertices[0].coords();

    if (npoints == 1)
    {
        // Simplex is a point
        return { pA, point - pA, Voronoi::Vertex };
    }

    vec2 pB = vertices[1].coords();
    auto [uAB, vAB, divAB] = nonNormalisedBary(point, pA, pB);

    if (npoints == 2)
    {
        // Simplex is a line segment
        if (uAB <= 0)
        {
            // removeA = true;
            vertices[0].markForRemoval();
            return { pB, point - pB, Voronoi::Vertex };
        }
        else if (vAB <= 0)
        {
            //removeB = true;
            vertices[1].markForRemoval();
            return { pA, point - pA, Voronoi::Vertex };
        }
        else
        {
            vec2 d = perp(pA - pB);

            if (dot(d, point - pA) < 0)
            {
                d = -d;
            }

            return { (uAB * pA + vAB * pB) / divAB, d, Voronoi::Edge };
        }
    }

    vec2 pC = vertices[2].coords();
    auto [uBC, vBC, divBC] = nonNormalisedBary(point, pB, pC);
    auto [uCA, vCA, divCA] = nonNormalisedBary(point, pC, pA);
    auto [uABC, vABC, wABC, divABC] = nonNormalisedBary(point, pA, pB, pC);

    if (npoints == 3)
    {
        // Simplex is a triangle
        if (vAB <= 0 && uCA <= 0)
        {
            //removeB = true;
            //removeC = true;
            vertices[1].markForRemoval();
            vertices[2].markForRemoval();
            return { pA, point - pA, Voronoi::Vertex };
        }
        else if (uAB <= 0 && vBC <= 0)
        {
            //removeA = true;
            //removeC = true;
            vertices[0].markForRemoval();
            vertices[2].markForRemoval();
            return { pB, point - pB, Voronoi::Vertex };
        }
        else if (uBC <= 0 && vCA <= 0)
        {
            //removeA = true;
            //removeB = true;
            vertices[0].markForRemoval();
            vertices[1].markForRemoval();
            return { pC, point - pC, Voronoi::Vertex };
        }
        else if (uAB > 0 && vAB > 0 && wABC * divABC <= 0)
        {
            //removeC = true;
            vertices[2].markForRemoval();

            vec2 d = perp(pA - pB);

            if (dot(d, point - pA) < 0)
            {
                d = -d;
            }

            return { (uAB * pA + vAB * pB) / divAB, d, Voronoi::Edge };
        }
        else if (uBC > 0 && vBC > 0 && uABC * divABC <= 0)
        {
            //removeA = true;
            vertices[0].markForRemoval();

            vec2 d = perp(pB - pC);

            if (dot(d, point - pB) < 0)
            {
                d = -d;
            }

            return { (uBC * pB + vBC * pC) / divBC, d, Voronoi::Edge };
        }
        else if (uCA > 0 && vCA > 0 && vABC * divABC <= 0)
        {
            //removeB = true;
            vertices[1].markForRemoval();

            vec2 d = perp(pC - pA);

            if (dot(d, point - pC) < 0)
            {
                d = -d;
            }

            return { (uCA * pC + vCA * pA) / divCA, d, Voronoi::Edge };
        }
        else
        {
            // Point is inside triangle
            return { point, {}, Voronoi::Inside };
        }
    }
}

void Simplex::addVertex(const Vertex* vertex)
{
    vertices.push_back(vertex);
}

void Simplex::cleanupVertices()
{
   /* for (auto it = vertices.begin(); it != vertices.end(); )
    {
        if (it->removeFlagSet())
        {
            it->unsetRemoveFlag();
            it = vertices.erase(it);
        }
        else
        {
            ++it;
        }
    }*/

    std::erase_if(vertices, [](const SimplexVertex& v) { return v.removeFlagSet(); });
}

bool Simplex::contains(const Vertex* vertex) const
{
    for (const SimplexVertex& v : vertices)
    {
        if (v.matches(vertex))
        {
            return true;
        }
    }

    return false;
}