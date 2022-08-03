#include "Constraint.h"

Constraint::Constraint(const PhysicsSettings& ps, std::initializer_list<RigidBody*> rigidBodies):
	ps(ps), rigidBodies(rigidBodies)
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](RigidBody* const rb)
	{
		rb->addConstraint(this);
	});
}

Constraint::~Constraint()
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](RigidBody* const rb)
	{
		rb->removeConstraint(this);
	});
}
