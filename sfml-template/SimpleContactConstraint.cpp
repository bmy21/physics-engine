#include "SimpleContactConstraint.h"

SimpleContactConstraint::SimpleContactConstraint(const PhysicsSettings& ps, RigidBody* rb1, RigidBody* rb2):
	e(ps.eDefault), mu(ps.muDefault), rfLength(ps.rfLengthDefault),
	ContactConstraint(ps, rb1, rb2)
{
}

void SimpleContactConstraint::init()
{
	initPoints();
	ncp = contactPoints.size();
	storeTargetVelocities();
}

void SimpleContactConstraint::correctVel()
{
	if (rollingFriction)
	{
		for (auto& cp : contactPoints)
		{
			solvePointRollFriction(cp);
		}
	}

	for (auto& cp : contactPoints)
	{
		solvePointFriction(cp);
	}

	// Try simultaneous solution of normal velocities first
	if (ncp == 2 && ps.simulSolveVel && wellConditionedVel)
	{
		if (simulSolveVel())
		{
			return;
		}
	}

	// At this point, simultaneous solution failed, either because the condition number was too high or 
	// because one of the accumulated impulses would become negative. So, resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		solvePointVel(cp);
	}
}

void SimpleContactConstraint::correctPos()
{
	// Try simultaneous solution first
	if (ncp == 2 && ps.simulSolvePos)
	{
		updateNormal();

		for (auto& cp : contactPoints)
		{
			rebuildPoint(cp);
			updateNormalFactors(cp);
		}

		prepareSimulSolver();

		if (wellConditionedPos)
		{
			simulSolvePos();
			return;
		}
	}

	// At this point, the condition number was too high, so resort to the iterative solution.
	for (auto& cp : contactPoints)
	{
		updateNormal();
		rebuildPoint(cp);
		updateNormalFactors(cp);
		solvePointPos(cp);
	}
}

void SimpleContactConstraint::warmStart()
{
	for (auto& cp : contactPoints)
	{
		warmStartPoint(cp);
	}
}

void SimpleContactConstraint::prepareVelSolver()
{
	updateNormal();
	updateTangent();

	for (auto& cp : contactPoints)
	{
		updateNormalFactors(cp);
		updateTangentFactors(cp);
	}

	if (ncp == 2 && ps.simulSolveVel)
	{
		prepareSimulSolver();
	}
}

void SimpleContactConstraint::getImpulsesFrom(ContactConstraint* other)
{
	if (!idsMatch(other))
	{
		return;
	}

	for (int i = 0; i < ncp; ++i)
	{
		for (int j = 0; j < other->ncp; ++j)
		{
			if (contactPoints[i].matches(other->contactPoints[j]))
			{
				contactPoints[i].lambda = other->contactPoints[j].lambda;
				contactPoints[i].fLambda = other->contactPoints[j].fLambda;
				contactPoints[i].fRollLambda = other->contactPoints[j].fRollLambda;
				break;
			}
		}
	}
}

void SimpleContactConstraint::draw(sf::RenderWindow& window, real fraction, bool debug, sf::Text* text)
{
	real normalHalfLengthPix = 6;
	real normalThickness = 2;

	for (const auto& cp : contactPoints)
	{
		vec2 pos = { cp.point.x * ps.pixPerUnit, cp.point.y * ps.pixPerUnit };
		drawThickLine(window, pos - normalHalfLengthPix * n, pos + normalHalfLengthPix * n, normalThickness, sf::Color::Black);

		if (debug && text)
		{
			text->setCharacterSize(40);
			text->setFillColor(sf::Color::Magenta);
			text->setString("\n\n" + cp.idAsString());
			text->setPosition(pos.x, pos.y);
			centre(*text);

			window.draw(*text);
		}
	}
}

void SimpleContactConstraint::solvePointRollFriction(ContactPoint& cp)
{
	real vDotGradCfRoll = rb2->angVel() - rb1->angVel();

	real dfRollLambda = 0;
	real denom = rb1->IInv() + rb2->IInv();

	if (denom != 0)
	{
		dfRollLambda = -vDotGradCfRoll / denom;
		dfRollLambda = std::clamp(cp.fRollLambda + dfRollLambda, -rfLength * cp.lambda, rfLength * cp.lambda) - cp.fRollLambda;
	}

	cp.fRollLambda += dfRollLambda;

	rb1->applyDeltaVel({}, -rb1->IInv() * dfRollLambda);
	rb2->applyDeltaVel({}, rb2->IInv() * dfRollLambda);
}

void SimpleContactConstraint::solvePointFriction(ContactPoint& cp)
{
	real vDotGradCf = dot(rb2->velocity() - rb1->velocity(), t) + cp.tCrossFactor2 * rb2->angVel() - cp.tCrossFactor1 * rb1->angVel();

	real dfLambda = 0;
	if (cp.tMassFactor != 0)
	{
		dfLambda = -vDotGradCf / cp.tMassFactor;
		dfLambda = std::clamp(cp.fLambda + dfLambda, -mu * cp.lambda, mu * cp.lambda) - cp.fLambda;
	}

	cp.fLambda += dfLambda;

	rb1->applyDeltaVel(-t * rb1->mInv() * dfLambda, -cp.tCrossFactor1 * rb1->IInv() * dfLambda);
	rb2->applyDeltaVel(t * rb2->mInv() * dfLambda, cp.tCrossFactor2 * rb2->IInv() * dfLambda);
}
