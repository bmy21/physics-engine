#pragma once
#include "ContactConstraint.h"
#include "ContactPoint.h"
#include "Circle.h"

class CircleCircleContact : public ContactConstraint
{
public:
	CircleCircleContact(Circle* c1, Circle* c2, const PhysicsSettings& ps);

private:
	void initPoints() override;
	void rebuildPoint(ContactPoint& cp) override;
	void updateNormal() override;

	const Circle* const c1;
	const Circle* const c2;
};

