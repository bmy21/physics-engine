#include "ContactConstraint.h"

ContactConstraint::ContactConstraint(const PhysicsSettings& ps):
	ps(ps)
{
	e = ps.eDefault;
	mu = ps.muDefault;
}
