#pragma once

#include "Utils.h"
#include "RigidBody.h"

class PolyPolyContact;

class ContactConstraint
{
public:
	virtual ~ContactConstraint();

	virtual void warmStart() = 0;
	virtual void correctVel() = 0;
	virtual void correctPos() = 0;
	virtual void draw(sf::RenderWindow& window, real pixPerUnit, real fraction, bool debug = false, sf::Text* text = nullptr) = 0;

	virtual bool matches(const ContactConstraint* other) const = 0;
	virtual bool matches(const PolyPolyContact* other) const = 0;

	virtual void rebuild() = 0;
	virtual void rebuildFrom(ContactConstraint* other) = 0;

	virtual void updateCache() = 0;

	int numPersist = 0;

protected:
	real mu = 0.5;
	real e = 0.5;

	real beta = 0.3;
	real slop = 0.005;

private:

};

