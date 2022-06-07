#pragma once
#include "ContactConstraint.h"

/*
class PolyCircleContact : public ContactConstraint
{
public:
	PolyCircleContact(ConvexPolygon* p, Circle* c, const PhysicsSettings& ps);

	void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) override;

	bool matches(const ContactConstraint* other) const override { return other->matches(this); }
	bool matches(const PolyPolyContact* other) const override;
	bool matches(const CircleCircleContact* other) const override { return false; }
	void onRebuildFrom(ContactConstraint* other) override;

private:
	void initPoints() override;
	void rebuildPoints() override;
	void updateNormal() override;

	// TODO: make these const pointers (in other CCs too)
	ConvexPolygon* p = nullptr;
	ConvexPolygon* c = nullptr;

	void checkAndAddPoint(ContactPoint& cp, const vec2& ref, real eps);
};
*/