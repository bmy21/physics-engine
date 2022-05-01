#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"
#include "ContactConstraint.h"
#include "DistanceConstraint.h"

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
	real dtPhysics = 1.0 / 100;
	real dtMax = 1.0 / 10;

	int velIter = 10;
	int posIter = 4;
	

	// TODO: Capitalisation style?
	std::vector<std::unique_ptr<RigidBody>> RigidBodies;
	std::vector<std::unique_ptr<Constraint>> Constraints;

	std::vector<std::unique_ptr<ContactConstraint>> ContactConstraints;
	std::vector<std::unique_ptr<ContactConstraint>> NewContactConstraints;
};

