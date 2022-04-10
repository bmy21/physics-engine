#pragma once

#include "Utils.h"

class RigidBody
{
public:
	RigidBody();

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real fraction) = 0;

	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	real mInv = 0, IInv = 0;

private:


};

