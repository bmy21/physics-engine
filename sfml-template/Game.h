#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"

class Game
{
public:
	Game();
	void run();

private:

	sf::RenderWindow window;
	sf::Clock frameTimer;

	bool vsync = true;
	int fpsLimit = 144;
	real dtPhysics = 1.0 / 100;
	real dtMax = 1.0 / 10;

	std::vector<std::unique_ptr<RigidBody>> RigidBodies;
};

