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
#include "PinConstraint.h"
#include "WeldConstraint.h"
#include "CarDefinition.h"



class Game
{
public:
	Game();
	void run();

	const int pixWidth = 1920 *1.25;
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
	
	void addChain(int nLinks, real linkWidth, real linkLength, vec2 start = { 0, 0 }, real linkmInv = 0, real angle = 0);
	void addSoftBody(vec2 minVertex, int nx, int ny, real xSpace, real ySpace, real particleRad, real particlemInv, real tOsc, real dampingRatio);
	void addCar(CarDefinition cd, vec2 pos);

	void addToAABBTree(RigidBody* rb);

	DistanceConstraint* addLimitedSpring(RigidBody* rb1, RigidBody* rb2, vec2 local1, vec2 local2, real dist, 
		real tOsc, real damping, real frac, bool relativeToRefPoints = false);

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

