#pragma once

#include "Utils.h"

class RigidBody
{
public:
	RigidBody();

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real fraction) = 0;

private:


};

