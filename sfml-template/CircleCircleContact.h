#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "Circle.h"

class CircleCircleContact : public ContactConstraint
{
public:
	CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps);

	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override { return false; }
	bool matches(const CircleCircleContact* other) const override;
	void rebuildFrom(ContactConstraint* other) override;

private:
	void rebuildPoints() override;
	void updateNormal() override;

	Circle* c1 = nullptr;
	Circle* c2 = nullptr;
};

