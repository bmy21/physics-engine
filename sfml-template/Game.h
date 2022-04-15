#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"
#include "ContactConstraint.h"

class Game
{
public:
	Game();
	void run();

	// 1 Physics unit = pixPerUnit pixels
	const real pixPerUnit = 120;

private:

	sf::RenderWindow window;
	sf::Clock frameTimer;

	sf::Font font;
	sf::Text text;

	bool vsync = true;
	int fpsLimit = 144;
	real dtPhysics = 1.0 / 200;
	real dtMax = 1.0 / 10;
	

	std::vector<std::unique_ptr<RigidBody>> RigidBodies;
	std::vector<std::unique_ptr<ContactConstraint>> ContactConstraints;
};

