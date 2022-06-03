#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"
#include "ContactConstraint.h"
#include "DistanceConstraint.h"
#include "SoftDistanceConstraint.h"
#include "MouseConstraint.h"
#include "MouseHandler.h"
#include "PhysicsSettings.h"

class Game
{
public:
	Game();
	void run();

	// 1 Physics unit = pixPerUnit pixels
	const real pixPerUnit = 120;
	const int pixWidth = 1920;
	const int pixHeight = 1080;

private:
	sf::RenderWindow window;
	sf::Clock frameTimer;

	sf::Font font;
	sf::Text text;

	bool vsync = true;
	int fpsLimit = 144;
	real dtMax = 1.0 / 10;

	void integrateVelocities();
	void integratePositions();

	void updateConstraints();
	void warmStart();

	void correctVelocities();
	void correctPositions();
	void detectCollisions();

	void setupMouseConstraint();
	void removeMouseConstraint();

	void addConvexPolygon(int nsides, real len, vec2 coords = {0, 0}, real mInv = 0);

	vec2 pixToCoords(const vec2& pix) const;
	vec2 pixToCoords(real xPix, real yPix) const { return pixToCoords({ xPix, yPix }); }

	MouseConstraint* mc = nullptr;

	PhysicsSettings ps;
	MouseHandler mh;
	
	std::vector<std::unique_ptr<RigidBody>> rigidBodies;
	std::vector<std::unique_ptr<Constraint>> constraints;

	std::vector<std::unique_ptr<ContactConstraint>> contactConstraints;
	std::vector<std::unique_ptr<ContactConstraint>> newContactConstraints;
};

