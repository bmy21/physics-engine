#pragma once

#include "Utils.h"

class RigidBody
{
public:
	RigidBody();

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction) = 0;

	void intVel(real dt);
	void intPos(real dt);

	vec2 pixCoords(real pixPerUnit) const;

	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;

	real mInv = 0, IInv = 0;

private:
	vec2 posPrev;
	real thetaPrev = 0;

};

