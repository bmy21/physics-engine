#pragma once

#include "Utils.h"
#include "RigidBody.h"


class Constraint
{
public:
	virtual void correctVel() = 0;
	virtual void correctPos() = 0;

private:

};

