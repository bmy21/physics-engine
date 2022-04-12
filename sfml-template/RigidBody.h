#pragma once

#include "Utils.h"

class RigidBody
{
public:
	RigidBody();

	virtual void update(real dt) = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	void moveTo(const vec2& p) { pos = posPrev = p; }
	void rotateTo(real t) { theta = thetaPrev = t; }

	void integrateVel(real dt);
	void integratePos(real dt);

	vec2 interpolatePos(real fraction) const;
	real interpolateTheta(real fraction) const;

	vec2 pos, vel, acc;
	real theta = 0, omega = 0, alpha = 0;
	real mInv = 0, IInv = 0;
	real grav = 0;

private:
	vec2 posPrev;
	real thetaPrev = 0;

};

