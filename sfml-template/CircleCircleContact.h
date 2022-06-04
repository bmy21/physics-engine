#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "Circle.h"

class CircleCircleContact : public ContactConstraint
{
public:
	CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps);

	void warmStart() override;
	void correctVel() override;
	void correctPos() override;
	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override { return false; }
	bool matches(const CircleCircleContact* other) const override;

	void rebuild() override;
	void rebuildFrom(ContactConstraint* other) override;

	void updateCache() override;

private:
	Circle* c1 = nullptr;
	Circle* c2 = nullptr;

	real vRelTarget = 0;
	ContactPoint cp;

	// Geometric data - cached to avoid recomputation
	vec2 n, t;
	real nCrossFactor1 = 0, nCrossFactor2 = 0;
	real tCrossFactor1 = 0, tCrossFactor2 = 0;
	real nMassFactor = 0, tMassFactor = 0;
};

