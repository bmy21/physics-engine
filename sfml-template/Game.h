#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "ConvexPolygon.h"
#include "Circle.h"
#include "ContactConstraint.h"
#include "MouseConstraint.h"
#include "DistanceConstraint.h"
#include "LineConstraint.h"
#include "AngleConstraint.h"
#include "MouseHandler.h"
#include "PhysicsSettings.h"
#include "AABBTree.h"

class Game
{
public:
	Game();
	void run();

	const int pixWidth = 1920;
	const int pixHeight = 1080;

private:
	sf::RenderWindow window;
	sf::Clock frameTimer;

	sf::Font font;
	sf::Text text;

	bool vsync = true;
	int fpsLimit = 165;
	real dtMax = 1.0 / 10;

	void integrateVelocities();
	void integratePositions();

	void prepareVelSolvers();
	void warmStart();

	void correctVelocities();
	void correctPositions();

	void updateCollidingPairs();
	void checkCollision(RigidBody* rb1, RigidBody* rb2);

	void removeClickedRigidBody();

	void handleConstraintRemoval();
	void handleRigidBodyRemoval();

	void setupMouseConstraint();
	void removeMouseConstraint();

	void showStats(real frameTime);

	ConvexPolygon* addConvexPolygon(int nsides, real len, vec2 coords = {0, 0}, real mInv = 0);
	ConvexPolygon* addConvexPolygon(const std::vector<vec2>& points, vec2 coords = { 0, 0 }, real mInv = 0);
	Circle* addCircle(real rad, vec2 coords = { 0, 0 }, real mInv = 0);
	void addToAABBTree(RigidBody* rb);

	vec2 pixToCoords(real xPix, real yPix) const;
	vec2 pixToCoords(const vec2& pix) const { return pixToCoords(pix.x, pix.y); }

	MouseConstraint* mc = nullptr;

	PhysicsSettings ps;
	MouseHandler mh;
	
	AABBTree tree;

	std::vector<std::unique_ptr<RigidBody>> rigidBodies;
	std::vector<std::unique_ptr<Constraint>> constraints;

	std::map<idPair, std::unique_ptr<ContactConstraint>> collidingPairs;
};

