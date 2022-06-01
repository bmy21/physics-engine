#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"
#include "ContactConstraint.h"
#include "DistanceConstraint.h"
#include "SoftDistanceConstraint.h"
#include "MouseConstraint.h"
#include "MouseHandler.h"

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
	real dtPhysics = 1.0 / 120;
	real dtMax = 1.0 / 10;

	int velIter = 12;
	int posIter = 4;

	void integrateVelocities();
	void integratePositions();

	void updateConstraints();
	void warmStart();

	void correctVelocities();
	void correctPositions();
	void detectCollisions();

	void setupMouseConstraint();
	void removeMouseConstraint();

	MouseConstraint* mc = nullptr;
	std::unique_ptr<MouseHandler> mh;
	
	std::vector<std::unique_ptr<RigidBody>> rigidBodies;
	std::vector<std::unique_ptr<Constraint>> constraints;

	std::vector<std::unique_ptr<ContactConstraint>> contactConstraints;
	std::vector<std::unique_ptr<ContactConstraint>> newContactConstraints;
};

