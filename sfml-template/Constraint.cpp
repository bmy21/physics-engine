#include "Constraint.h"
#include "RigidBody.h"

Constraint::Constraint(const PhysicsSettings& ps, std::initializer_list<RigidBody*> rigidBodies):
	ps(ps), rigidBodies(rigidBodies)
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](RigidBody* const rb)
	{
		rb->addConstraintToList(this);
	});
}

Constraint::~Constraint()
{
	std::for_each(std::execution::unseq, rigidBodies.begin(), rigidBodies.end(), [&](RigidBody* const rb)
	{
		rb->removeConstraintFromList(this);
	});
}
