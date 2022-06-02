#include "ContactConstraint.h"

void ContactConstraint::initialise(const PhysicsSettings* ps)
{
	this->ps = ps;
	e = ps->eDefault;
	mu = ps->muDefault;

	onInit();
}
