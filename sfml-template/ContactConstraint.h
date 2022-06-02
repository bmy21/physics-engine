#pragma once

#include "Utils.h"
#include "RigidBody.h"
#include "PhysicsSettings.h"

class PolyPolyContact;

class ContactConstraint
{
public:
	void initialise(const PhysicsSettings* ps);

	virtual void onInit() = 0;

	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void warmStart() = 0;
	virtual void updateCache() = 0;

	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	virtual bool matches(const ContactConstraint* other) const = 0;
	virtual bool matches(const PolyPolyContact* other) const = 0;

	virtual void rebuild() = 0;
	virtual void rebuildFrom(ContactConstraint* other) = 0;

	int numPersist = 0;

protected:
	real mu = 0;
	real e = 0;

	const PhysicsSettings* ps = nullptr;

private:

};

