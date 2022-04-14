#pragma once

#include "Utils.h"
#include "RigidBody.h"

class ContactConstraint
{
public:
	virtual ~ContactConstraint();

	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction) = 0;

private:
	//RigidBody* ref = nullptr;
	//RigidBody* inc = nullptr;

};

