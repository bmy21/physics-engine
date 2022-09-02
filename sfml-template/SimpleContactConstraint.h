#pragma once
#include "ContactConstraint.h"
#
class SimpleContactConstraint : public ContactConstraint
{
public:
	SimpleContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2);

	void init() override;
	void correctVel() override;
	void correctPos() override;
	void warmStart() override;
	void prepareVelSolver() override;
	void getImpulsesFrom(ContactConstraint* other) override;

	void draw(sf::RenderWindow& window, real fraction, bool debug = false, sf::Text* text = nullptr) override;


protected:
	// The details of the below functions depend on the specific types of rigid body involved
	virtual void initPoints() = 0;
	virtual void updateNormal() = 0;
	virtual void rebuildPoint(ContactPoint& cp) = 0;

	void enableRollingFriction() { rollingFriction = true; }
	void disableRollingFriction() { rollingFriction = false; }
	bool idsMatch(const ContactConstraint* other) const;

	std::vector<ContactPoint> contactPoints;
	int ncp = 0;

	// Collision normal & tangent (shared by all contact points)
	vec2 n, t;

private:
	void storeTargetVelocities();
	void updateTangent();
	void prepareSimulSolver();

	bool simulSolveVel();
	void simulSolvePos();

	void updateNormalFactors(ContactPoint& cp);
	void updateTangentFactors(ContactPoint& cp);

	void solvePointRollFriction(ContactPoint& cp);
	void solvePointFriction(ContactPoint& cp);
	void solvePointVel(ContactPoint& cp);
	void solvePointPos(ContactPoint& cp);
	void warmStartPoint(ContactPoint& cp);

	real mu = 0;
	real e = 0;

	bool rollingFriction = false;
	real rfLength = 0;

	// Cached data for simultaneous solution
	bool wellConditionedVel = false;
	bool wellConditionedPos = false;
	real A12 = 0, det = 0, norm = 0;
};