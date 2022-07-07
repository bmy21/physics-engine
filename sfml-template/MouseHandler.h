#pragma once
#include "Utils.h"
#include "PhysicsSettings.h"

class MouseHandler
{
public:
	MouseHandler(const sf::RenderWindow& window, const PhysicsSettings& ps);

	void update();
	vec2 pixCoords() const { return mPixCoords; }
	vec2 coords() const { return mCoords; }

private:
	const sf::RenderWindow& window;
	const PhysicsSettings& ps;
	vec2 mPixCoords, mCoords;
};

