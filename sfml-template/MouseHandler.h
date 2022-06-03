#pragma once
#include "Utils.h"

class MouseHandler
{
public:
	MouseHandler(const sf::RenderWindow& window, real pixPerUnit);

	void update();
	vec2 pixCoords() const { return mPixCoords; }
	vec2 coords() const { return mCoords; }

private:
	const sf::RenderWindow& window;

	// TODO: Handle case where pixPerUnit changes - include in PhysicsSettings?
	real pixPerUnit = 1;
	vec2 mPixCoords, mCoords;
};

